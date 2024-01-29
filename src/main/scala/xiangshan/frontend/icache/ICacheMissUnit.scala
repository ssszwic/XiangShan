/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.frontend.icache

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.IdRange
import freechips.rocketchip.tilelink.ClientStates._
import freechips.rocketchip.tilelink.TLPermissions._
import freechips.rocketchip.tilelink._
import xiangshan._
import xiangshan.cache._
import utils._
import utility._
import difftest._


abstract class ICacheMissUnitModule(implicit p: Parameters) extends XSModule
  with HasICacheParameters

abstract class ICacheMissUnitBundle(implicit p: Parameters) extends XSBundle
  with HasICacheParameters


class Demultiplexer[T <: Data](val gen: T, val n: Int) extends Module
{
  /** Hardware module that is used to sequence 1 producers into n consumer.
    * Priority is given to lower producer.
    */
  require(n >= 2)
  val io = IO(new Bundle {
    val in      = Flipped(DecoupledIO(gen))
    val out     = Vec(n, DecoupledIO(gen))
    val chosen  = Output(UInt(log2Ceil(n).W))
  })

  val grant = false.B +: (1 until n).map(i=> (0 until i).map(io.out(_).ready).reduce(_||_))
  for (i <- 0 until n) {
    io.out(i).bits := io.in.bits
    io.out(i).valid := !grant(i) && io.in.valid
  }

  io.in.ready := grant.last || io.out.last.ready
  io.chosen := PriorityEncoder(VecInit(io.out.map(_.ready)))
}


class MuxBundle[T <: Data](val gen: T, val n: Int) extends Module
{
  require(n >= 2)
  val io = IO(new Bundle {
    val sel     = Input(UInt(log2Ceil(n).W))
    val in      = Flipped(Vec(n, DecoupledIO(gen)))
    val out     = DecoupledIO(gen)
  })

  io.in   <> DontCare
  io.out  <> DontCare
  for (i <- 0 until n) {
    when(io.sel === i.U) {
      io.out <> io.in(i)
    }
  }
}


class ICacheMissReq(implicit p: Parameters) extends ICacheBundle {
  val blkPaddr  = UInt((PAddrBits - blockOffBits).W)
  val waymask   = UInt(nWays.W)
}


class ICacheMissResp(implicit p: Parameters) extends ICacheBundle {
  val blkPaddr  = UInt((PAddrBits - blockOffBits).W)
  val data      = UInt(blockBits.W)
  val corrupt   = Bool()
}


class LookUpMSHR(implicit p: Parameters) extends ICacheBundle {
  val blkPaddr  = Output(UInt((PAddrBits - blockOffBits).W))
  val hit       = Input(Bool())
}

class MSHRResp(implicit p: Parameters) extends ICacheBundle {
  val req_info = new ICacheMissReq
  val id       = UInt(log2Ceil(nFetchMshr + nPrefetchEntries).W)
}


class ICacheMSHR(edge: TLEdgeOut, isFetch: Boolean, ID: Int)(implicit p: Parameters) extends ICacheMissUnitModule {
  val io = IO(new Bundle {
    val fencei      = Input(Bool())
    val flush       = Input(Bool())
    val invalid     = Input(Bool())
    val req         = Flipped(DecoupledIO(new ICacheMissReq))
    val acquire     = DecoupledIO(new TLBundleA(edge.bundle))
    val lookUps     = Flipped(Vec(3, new LookUpMSHR))
    val resp        = ValidIO(new ICacheMissReq)
  })

  val valid     = RegInit(Bool(), false.B)
  // this MSHR doesn't respones to fetch and sram
  val flush     = RegInit(Bool(), false.B)
  val fencei    = RegInit(Bool(), false.B)
  // this MSHR has been issued
  val issue     = RegInit(Bool(), false.B)

  val blkPaddr  = RegInit(UInt((PAddrBits - blockOffBits).W), 0.U)
  val waymask   = RegInit(UInt(nWays.W),   0.U)

  // look up and return result at the same cycle
  val hits = io.lookUps.map(lookup => (lookup.blkPaddr === blkPaddr) && valid && !fencei)
  val hit = hits.reduce(_||_)
  (0 until 3).foreach { i =>
    io.lookUps(i).hit := hits(i)
  }

  // wake up when hit MSHR (fencei is low)
  when(hit) {
    flush := false.B
  }

  // invalid when the req hasn't been issued
  // invalid after grant finish when the req has been issued
  when(io.flush || io.fencei) {
    when(issue) {
      flush := true.B
    }.otherwise {
      valid := false.B
    }
  }

  when(io.fencei) {
    fencei := true.B
  }

  // receive request and register
  io.req.ready := !valid && !io.flush && !io.fencei
  when(io.req.fire) {
    valid     := true.B
    flush     := false.B
    issue     := false.B
    fencei    := false.B
    blkPaddr  := io.req.bits.blkPaddr
    waymask   := io.req.bits.waymask
  }

  // send request to L2
  io.acquire.valid := valid && !issue && !io.flush && !io.fencei
  val getBlock = edge.Get(
    fromSource  = ID.U,
    toAddress   = Cat(blkPaddr, 0.U(log2Ceil(blockBytes).W)),
    lgSize      = (log2Up(cacheParams.blockBytes)).U
  )._2
  io.acquire.bits := getBlock
  io.acquire.bits.user.lift(ReqSourceKey).foreach(_ := MemReqSource.CPUInst.id.U)
  when(io.acquire.fire) {
    issue := true.B
  }

  // invalid request when grant finish
  when(io.invalid) {
    valid := false.B
  }

  // offer the information other than data for write sram and response fetch
  io.resp.valid         := valid && (!flush || (flush && !fencei && hit))
  io.resp.bits.blkPaddr := blkPaddr
  io.resp.bits.waymask  := waymask
}


class ICacheMissBundle(edge: TLEdgeOut)(implicit p: Parameters) extends ICacheBundle{
  // difftest
  val hartId      = Input(Bool())
  // control
  val fencei      = Input(Bool())
  val flush       = Input(Bool())
  // fetch
  val fetch_req     = Flipped(DecoupledIO(new ICacheMissReq))
  val fetch_resp    = ValidIO(new ICacheMissResp)
  // prefetch
  val prefetch_req  = Flipped(DecoupledIO(new ICacheMissReq))
  // look up
  val lookUps       = Flipped(Vec(3, new LookUpMSHR))
  // SRAM Write Req
  val meta_write  = DecoupledIO(new ICacheMetaWriteBundle)
  val data_write  = DecoupledIO(new ICacheDataWriteBundle)
  // Tilelink
  val mem_acquire = DecoupledIO(new TLBundleA(edge.bundle))
  val mem_grant   = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))
}


class ICacheMissUnit(edge: TLEdgeOut)(implicit p: Parameters) extends ICacheMissUnitModule
{
  val io = IO(new ICacheMissBundle(edge))
                                     
  /**
    ******************************************************************************
    * fetch have higher priority
    * fetch MSHR: lower index have a higher priority
    * prefetch MSHR: the prefetchMSHRs earlier have a higher priority
    *                 ---------       --------------       -----------
    * ---fetch reg--->| Demux |-----> | fetch MSHR |------>| Arbiter |---acquire--->
    *                 ---------       --------------       -----------
    *                                 | fetch MSHR |            ^
    *                                 --------------            |
    *                                                           |
    *                                -----------------          |
    *                                | prefetch MSHR |          |
    *                 ---------      -----------------     ----------- 
    * ---fetch reg--->| Demux |----> | prefetch MSHR |---->| Arbiter |
    *                 ---------      -----------------     -----------
    *                                |    .......    |
    *                                -----------------
    ******************************************************************************
    */

  val fetchDemux    = Module(new Demultiplexer(new ICacheMissReq, nFetchMshr))
  val prefetchDemux = Module(new Demultiplexer(new ICacheMissReq, nPrefetchMshr))
  val prefetchArb   = Module(new MuxBundle(new TLBundleA(edge.bundle), nPrefetchMshr))
  val acquireArb    = Module(new Arbiter(new TLBundleA(edge.bundle), nFetchMshr + 1))
  
  fetchDemux.io.in         <> io.fetch_req
  prefetchDemux.io.in      <> io.prefetch_req
  io.mem_acquire           <> acquireArb.io.out
  acquireArb.io.in.last    <> prefetchArb.io.out

  val fetchMSHRs = (0 until nFetchMshr).map { i =>
    val mshr = Module(new ICacheMSHR(edge, true, i))
    mshr.io.flush   := io.flush
    mshr.io.fencei  := io.fencei
    mshr.io.req <> fetchDemux.io.out(i)
    (0 until 3).foreach { j =>
      mshr.io.lookUps(j).blkPaddr := io.lookUps(j).blkPaddr
    }
    acquireArb.io.in(i) <> mshr.io.acquire
    mshr
  }

  val prefetchMSHRs = (0 until nPrefetchMshr).map { i =>
    val mshr = Module(new ICacheMSHR(edge, false, nFetchMshr + i))
    mshr.io.flush := io.flush
    mshr.io.fencei  := io.fencei
    mshr.io.req <> prefetchDemux.io.out(i)
    (0 until 3).foreach { j =>
      mshr.io.lookUps(j).blkPaddr := io.lookUps(j).blkPaddr
    }
    prefetchArb.io.in(i) <> mshr.io.acquire
    mshr
  }

  /**
    ******************************************************************************
    * MSHR look up
    * - look up all mshr and return at the same cycle
    ******************************************************************************
    */
  val allMSHRs = (fetchMSHRs ++ prefetchMSHRs)
  (0 until 3).map { i =>
    io.lookUps(i).hit := allMSHRs.map(mshr => mshr.io.lookUps(i).hit).reduce(_||_)
  }

  /**
    ******************************************************************************
    * prefetchMSHRs priority
    * - The requests that enter the prefetchMSHRs earlier have a higher priority in issuing.
    * - The order of enqueuing is recorded in FIFO when requset enters MSHRs.
    * - The requests are dispatched in the order they are recorded in FIFO.
    ******************************************************************************
    */
  // When the FIFO is full, enqueue and dequeue operations do not occur at the same cycle.
  // So the depth of the FIFO is set to match the number of MSHRs.
  val priorityFIFO = Module(new Queue(UInt(log2Ceil(nPrefetchMshr).W), nPrefetchMshr, hasFlush=true))
  priorityFIFO.io.flush.get := io.flush || io.fencei
  priorityFIFO.io.enq.valid := io.prefetch_req.fire
  priorityFIFO.io.enq.bits  := prefetchDemux.io.chosen
  priorityFIFO.io.deq.ready := prefetchArb.io.out.fire
  prefetchArb.io.sel        := priorityFIFO.io.deq.bits
  assert(!(priorityFIFO.io.enq.fire ^ io.prefetch_req.fire), "priorityFIFO.io.enq and io.prefetch_req must fire at the same cycle")
  assert(!(priorityFIFO.io.deq.fire ^ prefetchArb.io.out.fire), "priorityFIFO.io.deq and prefetchArb.io.out must fire at the same cycle")

  /**
    ******************************************************************************
    * Tilelink D channel (grant)
    ******************************************************************************
    */
  //cacheline register
  val readBeatCnt = RegInit(UInt(log2Up(refillCycles).W), 0.U)
  val respDataReg = RegInit(VecInit(Seq.fill(refillCycles)(0.U(beatBits.W))))

  val wait_last = readBeatCnt === (refillCycles - 1).U
  when(io.mem_grant.fire && edge.hasData(io.mem_grant.bits)) {
    respDataReg(readBeatCnt) := io.mem_grant.bits.data
    readBeatCnt := Mux(wait_last || io.mem_grant.bits.corrupt, 0.U, readBeatCnt + 1.U)
  }

  // last transition finsh or corrupt
  val last_fire = io.mem_grant.fire && edge.hasData(io.mem_grant.bits) && 
                  (wait_last || io.mem_grant.bits.corrupt)

  val (_, _, refill_done, _) = edge.addr_inc(io.mem_grant)
  assert(!(refill_done ^ last_fire), "refill not done!")
  io.mem_grant.ready := true.B

  val last_fire_r = RegNext(last_fire)
  val id_r        = RegNext(io.mem_grant.bits.source)
  val corrupt_r   = RegNext(io.mem_grant.bits.corrupt)

  /**
    ******************************************************************************
    * invalid mshr when finish transition
    ******************************************************************************
    */
  (0 until (nFetchMshr + nPrefetchMshr)).foreach{ i =>
    allMSHRs(i).io.invalid := last_fire_r && (id_r === i.U)
  }

  /**
    ******************************************************************************
    * response fetch and write SRAM
    ******************************************************************************
    */
  // get request information from MSHRs
  val allMSHRs_resp = VecInit(allMSHRs.map(mshr => mshr.io.resp))
  val mshr_resp = allMSHRs_resp(id_r)


  // write SRAM
  io.meta_write.bits.generate(tag     = getPhyTagFromBlk(mshr_resp.bits.blkPaddr),
                              idx     = getIdxFromBlk(mshr_resp.bits.blkPaddr),
                              waymask = mshr_resp.bits.waymask,
                              bankIdx = getIdxFromBlk(mshr_resp.bits.blkPaddr)(0))
  io.data_write.bits.generate(data    = respDataReg.asUInt,
                              idx     = getIdxFromBlk(mshr_resp.bits.blkPaddr),
                              waymask = mshr_resp.bits.waymask,
                              bankIdx = getIdxFromBlk(mshr_resp.bits.blkPaddr)(0))
  val write_sram_valid = mshr_resp.valid && last_fire_r && !io.flush && !io.fencei && !corrupt_r
  io.meta_write.valid := write_sram_valid
  io.data_write.valid := write_sram_valid

  // response fetch
  io.fetch_resp.valid         := mshr_resp.valid && last_fire_r && !io.flush && !io.fencei
  io.fetch_resp.bits.blkPaddr := mshr_resp.bits.blkPaddr
  io.fetch_resp.bits.data     := respDataReg.asUInt
  io.fetch_resp.bits.corrupt  := corrupt_r

  /**
    ******************************************************************************
    * Difftest
    ******************************************************************************
    */
  if (env.EnableDifftest) {
    val difftest = DifftestModule(new DiffRefillEvent, dontCare = true)
    difftest.coreid := io.hartId
    difftest.index  := 0.U
    difftest.valid  := io.meta_write.valid
    difftest.addr   := Cat(mshr_resp.bits.blkPaddr, 0.U(blockBytes.W))
    difftest.data   := respDataReg.asTypeOf(difftest.data)
    difftest.idtfr  := DontCare
  }
}