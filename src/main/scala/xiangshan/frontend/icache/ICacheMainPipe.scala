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
import difftest._
import freechips.rocketchip.tilelink.ClientStates
import xiangshan._
import xiangshan.cache.mmu._
import utils._
import utility._
import xiangshan.backend.fu.{PMPReqBundle, PMPRespBundle}
import xiangshan.frontend.{FtqICacheInfo, FtqToICacheRequestBundle}

class ICacheMainPipeReq(implicit p: Parameters) extends ICacheBundle
{
  val vaddr  = UInt(VAddrBits.W)
  def vsetIdx = get_idx(vaddr)
}

class ICacheMainPipeResp(implicit p: Parameters) extends ICacheBundle
{
  val vaddr    = UInt(VAddrBits.W)
  // val registerData = UInt(blockBits.W)
  // val sramData = UInt(blockBits.W)
  // val select   = Bool()
  val data = UInt((blockBits/2).W)
  val paddr    = UInt(PAddrBits.W)
  val tlbExcp  = new Bundle{
    val pageFault = Bool()
    val accessFault = Bool()
    val mmio = Bool()
  }
}

class ICacheMainPipeBundle(implicit p: Parameters) extends ICacheBundle
{
  val req  = Flipped(Decoupled(new FtqToICacheRequestBundle))
  val resp = Vec(PortNumber, ValidIO(new ICacheMainPipeResp))
  val topdownIcacheMiss = Output(Bool())
  val topdownItlbMiss = Output(Bool())
}

class ICacheMetaReqBundle(implicit p: Parameters) extends ICacheBundle{
  val toIMeta       = DecoupledIO(new ICacheReadBundle)
  val fromIMeta     = Input(new ICacheMetaRespBundle)
}

class ICacheDataReqBundle(implicit p: Parameters) extends ICacheBundle{
  val toIData       = DecoupledIO(Vec(partWayNum, new ICacheReadBundle))
  val fromIData     = Input(new ICacheDataRespBundle)
}

class ICacheMSHRBundle(implicit p: Parameters) extends ICacheBundle{
  val req   = Decoupled(new ICacheMissReq)
  val resp  = Flipped(ValidIO(new ICacheMissResp))
}

class ICachePMPBundle(implicit p: Parameters) extends ICacheBundle{
  val req  = Valid(new PMPReqBundle())
  val resp = Input(new PMPRespBundle())
}

class ICachePerfInfo(implicit p: Parameters) extends ICacheBundle{
  val only_0_hit     = Bool()
  val only_0_miss    = Bool()
  val hit_0_hit_1    = Bool()
  val hit_0_miss_1   = Bool()
  val miss_0_hit_1   = Bool()
  val miss_0_miss_1  = Bool()
  val hit_0_except_1 = Bool()
  val miss_0_except_1 = Bool()
  val except_0       = Bool()
  val bank_hit       = Vec(2,Bool())
  val hit            = Bool()
}

class ICacheMainPipeInterface(implicit p: Parameters) extends ICacheBundle {
  val hartId = Input(UInt(8.W))
  /*** internal interface ***/
  val metaArray     = new ICacheMetaReqBundle
  val dataArray     = new ICacheDataReqBundle
  /** prefetch io */
  val touch = Vec(PortNumber,ValidIO(new ReplacerTouch))

  val mshr          = new ICacheMSHRBundle
  val errors        = Output(Vec(PortNumber, new L1CacheErrorInfo))
  /*** outside interface ***/
  //val fetch       = Vec(PortNumber, new ICacheMainPipeBundle)
  /* when ftq.valid is high in T + 1 cycle
   * the ftq component must be valid in T cycle
   */
  val fetch       = new ICacheMainPipeBundle
  val pmp         = Vec(PortNumber, new ICachePMPBundle)
  val itlb        = Vec(PortNumber, new TlbRequestIO)
  val respStall   = Input(Bool())

  val csr_parity_enable = Input(Bool())
  val flush = Input(Bool())

  val perfInfo = Output(new ICachePerfInfo)
}

class ICacheDB(implicit p: Parameters) extends ICacheBundle {
  val blk_vaddr   = UInt((VAddrBits - blockOffBits).W)
  val blk_paddr   = UInt((PAddrBits - blockOffBits).W)
  val hit         = Bool()
}

class ICacheMainPipe(implicit p: Parameters) extends ICacheModule
{
  val io = IO(new ICacheMainPipeInterface)

  /** Input/Output port */
  val (fromFtq, toIFU)    = (io.fetch.req,          io.fetch.resp)
  val (toMeta,  metaResp) = (io.metaArray.toIMeta,  io.metaArray.fromIMeta)
  val (toData,  dataResp) = (io.dataArray.toIData,  io.dataArray.fromIData)
  val (toMSHR,  fromMSHR) = (io.mshr.req,           io.mshr.resp)
  val (toITLB,  fromITLB) = (io.itlb.map(_.req),    io.itlb.map(_.resp))
  val (toPMP,   fromPMP)  = (io.pmp.map(_.req),     io.pmp.map(_.resp))

  // Statistics on the frequency distribution of FTQ fire interval
  val cntFtqFireInterval = RegInit(0.U(32.W))
  cntFtqFireInterval := Mux(fromFtq.fire, 1.U, cntFtqFireInterval + 1.U)
  XSPerfHistogram("ftq2icache_fire_" + p(XSCoreParamsKey).HartId.toString, 
                  cntFtqFireInterval, fromFtq.fire,
                  1, 300, 1, right_strict = true)

  // Ftq RegNext Register
  val fromFtqReq = fromFtq.bits.pcMemRead

  /** pipeline control signal */
  val s1_ready, s2_ready = Wire(Bool())
  val s0_fire,  s1_fire , s2_fire  = Wire(Bool())

  /**
    ******************************************************************************
    * ICache Stage 0
    * - send req to ITLB and wait for tlb miss fixing
    * - send req to Meta/Data SRAM
    ******************************************************************************
    */

  /** s0 control */
  val s0_valid       = fromFtq.valid
  val s0_req_vaddr   = (0 until partWayNum + 1).map(i => VecInit(Seq(fromFtqReq(i).startAddr, fromFtqReq(i).nextlineStart)))
  val s0_req_vsetIdx = (0 until partWayNum + 1).map(i => VecInit(s0_req_vaddr(i).map(get_idx(_))))
  val s0_only_first  = (0 until partWayNum + 1).map(i => fromFtq.bits.readValid(i) && !fromFtqReq(i).crossCacheline)
  val s0_double_line = (0 until partWayNum + 1).map(i => fromFtq.bits.readValid(i) && fromFtqReq(i).crossCacheline)

  val s0_final_valid        = s0_valid
  val s0_final_vaddr        = s0_req_vaddr.head
  val s0_final_double_line  = s0_double_line.head

  /** SRAM request */
  // 0,1,2,3 -> dataArray(data); 3 -> dataArray(code); 0 -> metaArray; 4 -> itlb
  val ftq_req_to_data_doubleline  = s0_double_line.init
  val ftq_req_to_data_vset_idx    = s0_req_vsetIdx.init
  val ftq_req_to_data_valid       = fromFtq.bits.readValid.init

  val ftq_req_to_meta_doubleline  = s0_double_line.head
  val ftq_req_to_meta_vset_idx    = s0_req_vsetIdx.head
  val ftq_req_to_meta_valid       = fromFtq.bits.readValid.head

  val ftq_req_to_itlb_doubleline  = s0_double_line.last
  val ftq_req_to_itlb_vaddr       = s0_req_vaddr.last

  /** Data request */
  for(i <- 0 until partWayNum) {
    toData.valid                  := ftq_req_to_data_valid(i)
    toData.bits(i).isDoubleLine   := ftq_req_to_data_doubleline(i)
    toData.bits(i).vSetIdx        := ftq_req_to_data_vset_idx(i)
  }

  /** Meta request */
  toMeta.valid               := ftq_req_to_meta_valid
  toMeta.bits.isDoubleLine   := ftq_req_to_meta_doubleline
  toMeta.bits.vSetIdx        := ftq_req_to_meta_vset_idx

  val toITLB_s0_valid    = VecInit(Seq(s0_valid, s0_valid && ftq_req_to_itlb_doubleline))
  val toITLB_s0_size     = VecInit(Seq(3.U, 3.U)) // TODO: fix the size
  val toITLB_s0_vaddr    = ftq_req_to_itlb_vaddr
  val toITLB_s0_debug_pc = ftq_req_to_itlb_vaddr

  val itlb_can_go    = toITLB(0).ready && toITLB(1).ready
  val icache_can_go  = toData.ready && toMeta.ready
  val pipe_can_go    = s1_ready
  val s0_can_go      = itlb_can_go && icache_can_go && pipe_can_go
  s0_fire  := s0_valid && s0_can_go && !io.flush

  //TODO: fix GTimer() condition
  fromFtq.ready := s0_can_go

  /**
    ******************************************************************************
    * ICache Stage 1
    * - get tlb resp data (exceptiong info and physical addresses)
    * - get Meta/Data SRAM read responses (latched for pipeline stop)
    * - tag compare/hit check
    * - monitor missUint response port
    ******************************************************************************
    */

  /** s1 control */
  val s1_valid = generatePipeControl(lastFire = s0_fire, thisFire = s1_fire, thisFlush = io.flush, lastFlush = false.B)

  val s1_req_vaddr   = RegEnable(s0_final_vaddr, s0_fire)
  val s1_req_vsetIdx = s1_req_vaddr.map(get_idx(_))
  val s1_double_line = RegEnable(s0_final_double_line, s0_fire)

  /**
    ******************************************************************************
    * tlb request and response
    ******************************************************************************
    */
  fromITLB.foreach(_.ready := true.B)
  val s1_wait_itlb  = RegInit(VecInit(Seq.fill(PortNumber)(false.B)))

  (0 until PortNumber).foreach { i =>
    when(io.flush) {
      s1_wait_itlb(i) := false.B
    }.elsewhen(RegNext(s0_fire) && fromITLB(i).bits.miss) {
      s1_wait_itlb(i) := true.B
    }.elsewhen(s1_wait_itlb(i) && !fromITLB(i).bits.miss) {
      s1_wait_itlb(i) := false.B
    }
  }

  val s1_need_itlb = Seq((RegNext(s0_fire) || s1_wait_itlb(0)) && fromITLB(0).bits.miss,
                         (RegNext(s0_fire) || s1_wait_itlb(1)) && fromITLB(1).bits.miss && s1_double_line)
  val toITLB_s1_valid    = s1_need_itlb
  val toITLB_s1_size     = VecInit(Seq(3.U, 3.U)) // TODO: fix the size
  val toITLB_s1_vaddr    = s1_req_vaddr
  val toITLB_s1_debug_pc = s1_req_vaddr

  // chose tlb req between s0 and s1
  for (i <- 0 until PortNumber) {
    toITLB(i).valid         := Mux(s1_need_itlb(i), toITLB_s1_valid(i), toITLB_s0_valid(i))
    toITLB(i).bits.size     := Mux(s1_need_itlb(i), toITLB_s1_size(i), toITLB_s0_size(i))
    toITLB(i).bits.vaddr    := Mux(s1_need_itlb(i), toITLB_s1_vaddr(i), toITLB_s0_vaddr(i))
    toITLB(i).bits.debug.pc := Mux(s1_need_itlb(i), toITLB_s1_debug_pc(i), toITLB_s0_debug_pc(i))
  }
  toITLB.map{port =>
    port.bits.cmd                 := TlbCmd.exec
    port.bits.memidx              := DontCare
    port.bits.debug.robIdx        := DontCare
    port.bits.no_translate        := false.B
    port.bits.debug.isFirstIssue  := DontCare
    port.bits.kill                := DontCare
  }
  io.itlb.foreach(_.req_kill := false.B)

  val tlb_valid_tmp = VecInit(Seq((RegNext(s0_fire) || s1_wait_itlb(0)) && !fromITLB(0).bits.miss,
                                  (RegNext(s0_fire) || s1_wait_itlb(1)) && !fromITLB(1).bits.miss && s1_double_line))
  val tlbRespPAddr  = VecInit((0 until PortNumber).map(i =>
                        ResultHoldBypass(valid = tlb_valid_tmp(i), data = fromITLB(i).bits.paddr(0))))
  val tlbExcpPF     = VecInit((0 until PortNumber).map(i =>
                        ResultHoldBypass(valid = tlb_valid_tmp(i), data = fromITLB(i).bits.excp(0).pf.instr)))
  val tlbExcpAF     = VecInit((0 until PortNumber).map(i =>
                        ResultHoldBypass(valid = tlb_valid_tmp(i), data = fromITLB(i).bits.excp(0).af.instr)))
  val tlbExcp       = VecInit((0 until PortNumber).map(i => tlbExcpAF(i) || tlbExcpPF(i)))

  val s1_tlb_valid = VecInit((0 until PortNumber).map(i => ValidHoldBypass(tlb_valid_tmp(i), s1_fire || io.flush)))
  val tlbRespAllValid = s1_tlb_valid(0) && (!s1_double_line || s1_double_line && s1_tlb_valid(1))

  def numOfStage = 3
  val itlbMissStage = RegInit(VecInit(Seq.fill(numOfStage - 1)(0.B)))
  itlbMissStage(0) := !tlbRespAllValid
  for (i <- 1 until numOfStage - 1) {
    itlbMissStage(i) := itlbMissStage(i - 1)
  }

  /**
    ******************************************************************************
    * s1 hit check/tag compare
    ******************************************************************************
    */
  val s1_req_paddr        = tlbRespPAddr
  val s1_req_ptags        = VecInit(s1_req_paddr.map(get_phy_tag(_)))

  val s1_meta_ptags       = ResultHoldBypass(data = metaResp.tags, valid = RegNext(s0_fire))
  val s1_meta_valids      = ResultHoldBypass(data = metaResp.entryValid, valid = RegNext(s0_fire))
  val s1_meta_errors      = ResultHoldBypass(data = metaResp.errors, valid = RegNext(s0_fire))

  val s1_SRAM_cachelines  = ResultHoldBypass(data = dataResp.datas, valid = RegNext(s0_fire))
  val s1_SRAM_errorBits   = ResultHoldBypass(data = dataResp.codes, valid = RegNext(s0_fire))

  val s1_tag_eq_vec       = VecInit((0 until PortNumber).map( p => VecInit((0 until nWays).map( w =>  s1_meta_ptags(p)(w) ===  s1_req_ptags(p)))))
  val s1_tag_match_vec    = VecInit((0 until PortNumber).map( k => VecInit(s1_tag_eq_vec(k).zipWithIndex.map{ case(way_tag_eq, w) => way_tag_eq && s1_meta_valids(k)(w)})))
  val s1_SRAM_hit         = VecInit(Seq(s1_valid && ParallelOR(s1_tag_match_vec(0)), s1_valid && ParallelOR(s1_tag_match_vec(1)) && s1_double_line))

  /**
    ******************************************************************************
    * monitor missUint response port
    ******************************************************************************
    */
  val fromMSHR_dataVec    = fromMSHR.bits.data.asTypeOf(Vec(2, UInt((blockBits/2).W)))
  val s1_MSHR_cacheline   = Wire(Vec(2,  UInt((blockBits/2).W)))
  val s1_MSHR_match       = VecInit((0 until PortNumber).map( i => (s1_req_vsetIdx(i) === fromMSHR.bits.vsetIdx) &&
                                                             (s1_req_ptags(i) === getPhyTagFromBlk(fromMSHR.bits.blkPaddr)) &&
                                                             s1_valid && fromMSHR.valid && !fromMSHR.bits.corrupt && s1_tlb_valid(i)))
  val s1_MSHR_hit = VecInit((0 until PortNumber).map(i => ValidHoldBypass(s1_MSHR_match(i), s1_fire || io.flush)))
  s1_MSHR_cacheline(0) := ResultHoldBypass(data = Mux(s1_double_line, fromMSHR_dataVec(1), fromMSHR_dataVec(0)), valid = s1_MSHR_match(0))
  s1_MSHR_cacheline(1) := ResultHoldBypass(data = Mux(s1_double_line, fromMSHR_dataVec(0), fromMSHR_dataVec(1)), valid = s1_MSHR_match(1) || (!s1_double_line && s1_MSHR_match(0)))

  when(s1_fire){
    assert(PopCount(s1_tag_match_vec(0)) <= 1.U && (PopCount(s1_tag_match_vec(1)) <= 1.U || !s1_double_line),
      "Multiple hit in main pipe, port0:is=%d,ptag=0x%x,vidx=0x%x,vaddr=0x%x port1:is=%d,ptag=0x%x,vidx=0x%x,vaddr=0x%x ",
      PopCount(s1_tag_match_vec(0)) > 1.U,s1_req_ptags(0), get_idx(s1_req_vaddr(0)), s1_req_vaddr(0),
      PopCount(s1_tag_match_vec(1)) > 1.U && s1_double_line, s1_req_ptags(1), get_idx(s1_req_vaddr(1)), s1_req_vaddr(1))
  }

  s1_ready := (s2_ready && tlbRespAllValid) || !s1_valid
  s1_fire  := s1_valid && tlbRespAllValid && s2_ready && !io.flush

  // record cacheline log
  // val isWriteICacheTable = WireInit(Constantin.createRecord("isWriteICacheTable" + p(XSCoreParamsKey).HartId.toString))
  // val ICacheTable = ChiselDB.createTable("ICacheTable" + p(XSCoreParamsKey).HartId.toString, new ICacheDB)

  // val ICacheDumpData_req0 = Wire(new ICacheDB)
  // ICacheDumpData_req0.blk_paddr := getBlkAddr(s1_req_paddr(0))
  // ICacheDumpData_req0.blk_vaddr := getBlkAddr(s1_req_vaddr(0))
  // ICacheDumpData_req0.hit       := s1_port_hit(0) || s1_prefetch_hit(0)
  // ICacheTable.log(
  //   data = ICacheDumpData_req0,
  //   en = isWriteICacheTable.orR && s1_fire,
  //   clock = clock,
  //   reset = reset
  // )

  // val ICacheDumpData_req1 = Wire(new ICacheDB)
  // ICacheDumpData_req1.blk_paddr := getBlkAddr(s1_req_paddr(1))
  // ICacheDumpData_req1.blk_vaddr := getBlkAddr(s1_req_vaddr(1))
  // ICacheDumpData_req1.hit       := s1_port_hit(1) || s1_prefetch_hit(1)
  // ICacheTable.log(
  //   data = ICacheDumpData_req1,
  //   en = isWriteICacheTable.orR && s1_fire && s1_double_line,
  //   clock = clock,
  //   reset = reset
  // )

  /** <PERF> replace victim way number */

  // (0 until nWays).map{ w =>
  //   XSPerfAccumulate("line_0_hit_way_" + Integer.toString(w, 10),  s1_fire && s1_port_hit(0) && OHToUInt(s1_tag_match_vec(0))  === w.U)
  // }

  // (0 until nWays).map{ w =>
  //   XSPerfAccumulate("line_0_victim_way_" + Integer.toString(w, 10),  s1_fire && !s1_port_hit(0) && OHToUInt(s1_victim_oh(0))  === w.U)
  // }

  // (0 until nWays).map{ w =>
  //   XSPerfAccumulate("line_1_hit_way_" + Integer.toString(w, 10),  s1_fire && s1_double_line && s1_port_hit(1) && OHToUInt(s1_tag_match_vec(1))  === w.U)
  // }

  // (0 until nWays).map{ w =>
  //   XSPerfAccumulate("line_1_victim_way_" + Integer.toString(w, 10),  s1_fire && s1_double_line && !s1_port_hit(1) && OHToUInt(s1_victim_oh(1))  === w.U)
  // }

  /**
    ******************************************************************************
    * ICache Stage 2
    * - send request to MSHR if ICache miss
    * - generate secondary miss status/data registers
    * - response to IFU
    ******************************************************************************
    */

  /** s2 control */
  val s2_fetch_finish = Wire(Bool())

  val s2_valid = generatePipeControl(lastFire = s1_fire, thisFire = s2_fire, thisFlush = io.flush, lastFlush = false.B)

  s2_ready  := (s2_valid && s2_fetch_finish && !io.respStall) || !s2_valid
  s2_fire   := s2_valid && s2_fetch_finish && !io.respStall && !io.flush

  /** s2 data */
  // val mmio = fromPMP.map(port => port.mmio) // TODO: handle it
  val s2_req_paddr        = RegEnable(s1_req_paddr,         s1_fire)
  val s2_req_vaddr        = RegEnable(s1_req_vaddr,         s1_fire)
  val s2_req_vsetIdx      = s2_req_vaddr.map(get_idx(_))
  val s2_double_line      = RegEnable(s1_double_line,       s1_fire)

  val s2_tag_match_vec    = RegEnable(s1_tag_match_vec,     s1_fire)
  val s2_SRAM_cachelines  = RegEnable(s1_SRAM_cachelines,   s1_fire)
  val s2_SRAM_hit         = RegEnable(s1_SRAM_hit,          s1_fire)

  val s2_MSHR_hit         = RegEnable(s1_MSHR_hit,          s1_fire)
  val s2_MSHR_cacheline   = RegEnable(s1_MSHR_cacheline,    s1_fire)

  val s2_meta_errors      = RegEnable(s1_meta_errors,       s1_fire)
  val s2_data_errorBits   = RegEnable(s1_SRAM_errorBits,    s1_fire)

  assert(RegNext(!s2_valid || s2_req_paddr(0)(11,0) === s2_req_vaddr(0)(11,0), true.B))

  /**
    ******************************************************************************
    * tlb exception and pmp logic
    ******************************************************************************
    */
  // short delay exception signal
  val s2_except_tlb_pf  = RegEnable(tlbExcpPF, s1_fire)
  val s2_except_tlb_af  = RegEnable(tlbExcpAF, s1_fire)
  val s2_except_tlb     = VecInit(Seq(s2_except_tlb_pf(0) || s2_except_tlb_af(0), s2_double_line && (s2_except_tlb_pf(1) || s2_except_tlb_af(1))))
  val s2_has_except_tlb = s2_valid && s2_except_tlb.reduce(_||_)
  // long delay exception signal
  // exception information and mmio
  val pmpExcpAF = VecInit(Seq(fromPMP(0).instr, fromPMP(1).instr && s2_double_line))
  val s2_except_pmp_af = DataHoldBypass(pmpExcpAF, RegNext(s1_fire))
  val s2_mmio = s2_valid && DataHoldBypass(fromPMP(0).mmio && !s2_except_tlb(0) && !s2_except_pmp_af(0), RegNext(s1_fire)).asBool
  // pmp port
  toPMP.zipWithIndex.map { case (p, i) =>
    p.valid     := s2_valid
    p.bits.addr := s2_req_paddr(i)
    p.bits.size := 3.U // TODO
    p.bits.cmd  := TlbCmd.exec
  }

  /**
    ******************************************************************************
    * monitor missUint response port
    ******************************************************************************
    */
  val curr_MSHR_hit       = RegInit(VecInit(Seq.fill(PortNumber)(false.B)))
  val curr_MSHR_cacheline = RegInit(VecInit(Seq.fill(PortNumber)(0.U((blockBits/2).W))))
  val curr_MSHR_corrupt   = RegInit(VecInit(Seq.fill(PortNumber)(false.B)))
  val curr_MSHR_match     = VecInit((0 until PortNumber).map( i =>
                              (if(i == 0) true.B else s2_double_line) &&
                              s2_valid && fromMSHR.valid && (s2_req_vsetIdx(i) === fromMSHR.bits.vsetIdx) &&
                              (get_phy_tag(s2_req_paddr(i)) === getPhyTagFromBlk(fromMSHR.bits.blkPaddr)))
                            )

  (0 until PortNumber).foreach{ i =>
    when(io.flush || s2_fire) {
      curr_MSHR_hit(i) := false.B
    }.elsewhen(curr_MSHR_match(i)) {
      curr_MSHR_hit(i) := true.B
      curr_MSHR_corrupt(i) := fromMSHR.bits.corrupt
    }
  }

  when(curr_MSHR_match(0)) {
    curr_MSHR_cacheline(0) := Mux(s2_double_line, fromMSHR_dataVec(1), fromMSHR_dataVec(0))
  }
  when(curr_MSHR_match(1) || (!s2_double_line && curr_MSHR_match(0))) {
    curr_MSHR_cacheline(1) := Mux(s2_double_line, fromMSHR_dataVec(0), fromMSHR_dataVec(1))
  }

  /**
    ******************************************************************************
    * miss handle (send MSHR req to missUnit)
    ******************************************************************************
    */
  // only handle port0 miss when port1 have tlb except or pmp except
  val s2_miss = Wire(Vec(PortNumber, Bool()))
  s2_miss(0) := !s2_SRAM_hit(0) && !s2_MSHR_hit(0) && !curr_MSHR_hit(0) && !s2_except_tlb(0) && !s2_except_pmp_af(0) && !s2_mmio
  s2_miss(1) := !s2_SRAM_hit(1) && !s2_MSHR_hit(1) && !curr_MSHR_hit(1) && s2_double_line && !s2_except_tlb(0) &&
                !s2_except_tlb(1) && !s2_except_pmp_af(0) && !s2_except_pmp_af(1) && !s2_mmio

  val toMSHRArbiter = Module(new Arbiter(new ICacheMissReq, PortNumber))

  // To avoid sending duplicate requests.
  val has_send = RegInit(VecInit(Seq.fill(2)(false.B)))
  (0 until PortNumber).foreach{ i =>
    when(s2_fire || io.flush) {
      has_send(i) := false.B
    }.elsewhen(toMSHRArbiter.io.in(i).fire) {
      has_send(i) := true.B
    }
  }

  (0 until PortNumber).map{ i =>
    toMSHRArbiter.io.in(i).valid          := s2_valid && s2_miss(i) && !has_send(i) && !io.flush
    toMSHRArbiter.io.in(i).bits.blkPaddr  := getBlkAddr(s2_req_paddr(i))
    toMSHRArbiter.io.in(i).bits.vsetIdx   := s2_req_vsetIdx(i)
  }

  toMSHR <> toMSHRArbiter.io.out

  XSPerfAccumulate("to_missUnit_stall",  toMSHR.valid && !toMSHR.ready)

  /**
    ******************************************************************************
    * select data from s2_SRAM, s2_MSHR and curr_MSHR
    ******************************************************************************
    */
  val s2_SRAM_cacheline = Wire(Vec(2, UInt((blockBits/2).W)))
  s2_SRAM_cacheline(0) := Mux1H(s2_tag_match_vec(0).asUInt, s2_SRAM_cachelines(0))
  s2_SRAM_cacheline(1) := Mux1H(Mux(s2_double_line, s2_tag_match_vec(1).asUInt, s2_tag_match_vec(0).asUInt), s2_SRAM_cachelines(1))

  val s2_fetch_data = Wire(Vec(2, UInt((blockBits/2).W)))
  s2_fetch_data(0) := Mux(s2_SRAM_hit(0), s2_SRAM_cacheline(0), Mux(s2_MSHR_hit(0), s2_MSHR_cacheline(0), curr_MSHR_cacheline(0)))
  s2_fetch_data(1) := Mux((s2_SRAM_hit(0) && !s2_double_line) || s2_SRAM_hit(1), s2_SRAM_cacheline(1), 
                          Mux((s2_MSHR_hit(0) && !s2_double_line) || s2_MSHR_hit(1), s2_MSHR_cacheline(1), curr_MSHR_cacheline(1)))

  s2_fetch_finish := !s2_miss.reduce(_||_)

  when(s2_fire){
    assert((s2_SRAM_hit(0).asUInt + s2_MSHR_hit(0).asUInt + curr_MSHR_hit(0).asUInt) <= 1.U &&
           ((s2_SRAM_hit(1).asUInt + s2_MSHR_hit(1).asUInt + curr_MSHR_hit(1).asUInt) <= 1.U || !s2_double_line),
      "Multiple hit in SRAM, s2_MSHR and curr_MSHR, port0:ptag=0x%x,vidx=0x%x,vaddr=0x%x port1:ptag=0x%x,vidx=0x%x,vaddr=0x%x ",
      get_phy_tag(s2_req_paddr(0)), get_idx(s2_req_vaddr(0)), s2_req_vaddr(0),
      get_phy_tag(s2_req_paddr(1)), get_idx(s2_req_vaddr(1)), s2_req_vaddr(1))
  }

  /**
    ******************************************************************************
    * IFU data resp
    ******************************************************************************
    */
  (0 until PortNumber).map{ i =>
    if(i ==0) toIFU(i).valid          := s2_fire
      else   toIFU(i).valid           := s2_fire && s2_double_line
    toIFU(i).bits.paddr               := s2_req_paddr(i)
    toIFU(i).bits.vaddr               := s2_req_vaddr(i)
    toIFU(i).bits.data                := s2_fetch_data(i)
    toIFU(i).bits.tlbExcp.pageFault   := s2_except_tlb_pf(i)
    toIFU(i).bits.tlbExcp.accessFault := s2_except_tlb_af(i) || curr_MSHR_corrupt(i) || s2_except_pmp_af(i)
    toIFU(i).bits.tlbExcp.mmio        := s2_mmio
  }

  /**
    ******************************************************************************
    * error resp: MSHR error
    ******************************************************************************
    */
  // data/meta parity error
  val s2_data_errors = Wire(Vec(PortNumber,Vec(nWays, Bool())))
  (0 until PortNumber).map{ i =>
    val read_datas = s2_SRAM_cachelines(i).asTypeOf(Vec(nWays,Vec(dataCodeUnitNum, UInt(dataCodeUnit.W))))
    val read_codes = s2_data_errorBits(i).asTypeOf(Vec(nWays,Vec(dataCodeUnitNum, UInt(dataCodeBits.W))))
    val data_full_wayBits = VecInit((0 until nWays).map( w =>
                                  VecInit((0 until dataCodeUnitNum).map( u =>
                                        Cat(read_codes(w)(u), read_datas(w)(u))))))
    val data_error_wayBits = VecInit((0 until nWays).map( w =>
                                  VecInit((0 until dataCodeUnitNum).map( u =>
                                       cacheParams.dataCode.decode(data_full_wayBits(w)(u)).error))))
    // register for timing
    if(i == 0){
      (0 until nWays).map{ w =>
        s2_data_errors(i)(w) := RegNext(RegNext(s1_fire)) && RegNext(data_error_wayBits(w)).reduce(_||_)
      }
    } else {
      (0 until nWays).map{ w =>
        s2_data_errors(i)(w) := RegNext(RegNext(s1_fire)) && RegNext(RegNext(s1_double_line)) && RegNext(data_error_wayBits(w)).reduce(_||_)
      } 
    }
  }

  val s2_parity_meta_error  = VecInit((0 until PortNumber).map(i => s2_meta_errors(i).reduce(_||_) && io.csr_parity_enable))
  val s2_parity_data_error  = VecInit((0 until PortNumber).map(i => s2_data_errors(i).reduce(_||_) && io.csr_parity_enable))
  val s2_parity_error       = VecInit((0 until PortNumber).map(i => RegNext(s2_parity_meta_error(i)) || s2_parity_data_error(i)))

  for(i <- 0 until PortNumber){
    io.errors(i).valid            := RegNext(s2_parity_error(i) && RegNext(RegNext(s1_fire)))
    io.errors(i).report_to_beu    := RegNext(s2_parity_error(i) && RegNext(RegNext(s1_fire)))
    io.errors(i).paddr            := RegNext(RegNext(s2_req_paddr(i)))
    io.errors(i).source           := DontCare
    io.errors(i).source.tag       := RegNext(RegNext(s2_parity_meta_error(i)))
    io.errors(i).source.data      := RegNext(s2_parity_data_error(i))
    io.errors(i).source.l2        := false.B
    io.errors(i).opType           := DontCare
    io.errors(i).opType.fetch     := true.B
  }

  // MSHR error
  (0 until PortNumber).map{ i =>
    when(RegNext(s2_fire && curr_MSHR_corrupt(i))){
      io.errors(i).valid            := true.B
      io.errors(i).report_to_beu    := false.B // l2 should have report that to bus error unit, no need to do it again
      io.errors(i).paddr            := RegNext(s2_req_paddr(i))
      io.errors(i).source.tag       := false.B
      io.errors(i).source.data      := false.B
      io.errors(i).source.l2        := true.B
    }
  }

  /**
    ******************************************************************************
    * update replacement status register
    ******************************************************************************
    */
  (0 until PortNumber).foreach{ i =>
    io.touch(i).bits.vsetIdx  := s2_req_vsetIdx(i)
    io.touch(i).bits.way      := OHToUInt(s2_tag_match_vec(i))
  }
  io.touch(0).valid := RegNext(s1_fire) && s2_SRAM_hit(0)
  io.touch(1).valid := RegNext(s1_fire) && s2_SRAM_hit(1) && s2_double_line

  // record ICache touch log

  class ICacheTouchDB(implicit p: Parameters) extends ICacheBundle{
    val blkPaddr  = UInt((PAddrBits - blockOffBits).W)
    val vsetIdx   = UInt(idxBits.W)
    val waymask   = UInt(log2Ceil(nWays).W)
  }

  val isWriteICacheTouchTable = WireInit(Constantin.createRecord("isWriteICacheTouchTable" + p(XSCoreParamsKey).HartId.toString))
  val ICacheTouchTable = ChiselDB.createTable("ICacheTouchTable" + p(XSCoreParamsKey).HartId.toString, new ICacheTouchDB)

  val ICacheTouchDumpData = Wire(Vec(PortNumber, new ICacheTouchDB))
  (0 until PortNumber).foreach{ i =>
    ICacheTouchDumpData(i).blkPaddr  := getBlkAddr(s2_req_paddr(i))
    ICacheTouchDumpData(i).vsetIdx   := s2_req_vsetIdx(i)
    ICacheTouchDumpData(i).waymask   := OHToUInt(s2_tag_match_vec(i))
    ICacheTouchTable.log(
      data  = ICacheTouchDumpData(i),
      en    = io.touch(i).valid,
      site  = "req_" + i.toString,
      clock = clock,
      reset = reset
    )
  }

  /**
    ******************************************************************************
    * performance info. TODO: need to simplify the logic
    ***********************************************************s*******************
    */
  val s2_fixed_hit = (0 until PortNumber).map(i=> s2_SRAM_hit(i) || s2_MSHR_hit(i))

  io.fetch.topdownIcacheMiss := s2_miss(0) || s2_miss(1)
  io.fetch.topdownItlbMiss := itlbMissStage(0)

  io.perfInfo.only_0_hit      :=  s2_fixed_hit(0) && !s2_double_line
  io.perfInfo.only_0_miss     := !s2_fixed_hit(0) && !s2_double_line
  io.perfInfo.hit_0_hit_1     :=  s2_fixed_hit(0) &&  s2_fixed_hit(1) && s2_double_line
  io.perfInfo.hit_0_miss_1    :=  s2_fixed_hit(0) && !s2_fixed_hit(1) && s2_double_line
  io.perfInfo.miss_0_hit_1    := !s2_fixed_hit(0) &&  s2_fixed_hit(1) && s2_double_line
  io.perfInfo.miss_0_miss_1   := !s2_fixed_hit(0) && !s2_fixed_hit(1) && s2_double_line
  io.perfInfo.hit_0_except_1  :=  s2_fixed_hit(0) && (s2_except_tlb(1) || s2_except_pmp_af(1)) && s2_double_line
  io.perfInfo.miss_0_except_1 := !s2_fixed_hit(0) && (s2_except_tlb(1) || s2_except_pmp_af(1)) && s2_double_line
  io.perfInfo.bank_hit(0)     :=  s2_fixed_hit(0)
  io.perfInfo.bank_hit(1)     :=  s2_fixed_hit(1) && s2_double_line
  io.perfInfo.except_0        :=  s2_except_tlb(0) || s2_except_pmp_af(0)
  io.perfInfo.hit             :=  s2_fixed_hit(0) && (!s2_double_line || s2_fixed_hit(1))

  /** <PERF> fetch bubble generated by icache miss*/
  XSPerfAccumulate("icache_bubble_s2_miss", s2_valid && !s2_fetch_finish )
  XSPerfAccumulate("icache_bubble_s0_tlb_miss", s1_valid && !tlbRespAllValid)

  XSPerfAccumulate("icache_port0_req", RegNext(s1_fire))
  XSPerfAccumulate("icache_port1_req", RegNext(s1_fire) && s2_double_line)

  (0 until PortNumber).foreach{ i =>
    XSPerfAccumulate(s"icache_port${i}_hit",       RegNext(s1_fire) && s2_fixed_hit(i))
    XSPerfAccumulate(s"icache_port${i}_last_hit",  RegNext(s1_fire) && s2_MSHR_hit(i))
    XSPerfAccumulate(s"icache_port${i}_miss",      RegNext(s1_fire) && s2_miss(i))
  }

  XSPerfAccumulate("port1_req", s2_fire && s2_double_line)

  /**
    ******************************************************************************
    * difftest refill check
    ******************************************************************************
    */
  if (env.EnableDifftest) {
    val discards = (0 until PortNumber).map { i =>
      val discard = toIFU(i).bits.tlbExcp.pageFault || toIFU(i).bits.tlbExcp.accessFault || toIFU(i).bits.tlbExcp.mmio
      discard
    }
    (0 until PortNumber).map { i =>
      val diffMainPipeOut = DifftestModule(new DiffRefillEvent, dontCare = true)
      diffMainPipeOut.coreid := io.hartId
      diffMainPipeOut.index := (3 + i).U
      if (i == 0) {
        diffMainPipeOut.valid := s2_fire && !discards(0)
        diffMainPipeOut.addr  := s2_req_paddr(0)
      } else {
        diffMainPipeOut.valid := s2_fire && !discards(0) && (!s2_double_line || (s2_double_line && !discards(1)))
        diffMainPipeOut.addr  := Mux(s2_double_line,
                                     Cat(getBlkAddr(s2_req_paddr(1)), 0.U(blockOffBits.W)),
                                     s2_req_paddr(0) + (blockBytes/2).U)
      }
      diffMainPipeOut.data := Cat(0.U((blockBits/2).W), toIFU(i).bits.data).asTypeOf(diffMainPipeOut.data)
      diffMainPipeOut.idtfr := DontCare
      diffMainPipeOut
    }
  }
}
