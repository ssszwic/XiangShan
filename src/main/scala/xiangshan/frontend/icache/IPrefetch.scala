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
import freechips.rocketchip.tilelink._
import utils._
import xiangshan.cache.mmu._
import xiangshan.frontend._
import xiangshan.backend.fu.{PMPReqBundle, PMPRespBundle}
import huancun.PreferCacheKey
import xiangshan.XSCoreParamsKey
import utility._

abstract class IPrefetchBundle(implicit p: Parameters) extends ICacheBundle
abstract class IPrefetchModule(implicit p: Parameters) extends ICacheModule

class IPredfetchIO(implicit p: Parameters) extends IPrefetchBundle {
  val ftqReq            = Flipped(new FtqPrefechBundle)
  val iTLBInter         = new TlbRequestIO
  val pmp               = new ICachePMPBundle
  val metaReadReq       = Decoupled(new PrefetchMetaReadBundle)
  val metaReadResp      = Input(new PrefetchMetaRespBundle)
  val prefetchReq       = DecoupledIO(new ICacheMissReq)
  val prefetch_req_hit  = Input(Bool())
  val MSHRResp          = Flipped(ValidIO(new ICacheMissResp))
  val flush             = Input(Bool())
  val csr_pf_enable     = Input(Bool())
}

class IPrefetchPipe(implicit p: Parameters) extends  IPrefetchModule
{
  val io = IO(new IPredfetchIO)

  val fromFtq = io.ftqReq
  val (toITLB,  fromITLB) = (io.iTLBInter.req, io.iTLBInter.resp)
  val (toPMP,  fromPMP)   = (io.pmp.req, io.pmp.resp)
  val (toIMeta, fromIMeta, fromIMetaValid) = (io.metaReadReq, io.metaReadResp.metaData, io.metaReadResp.entryValid)
  val fromMSHR  = io.MSHRResp
  val toMSHR    = io.prefetchReq

  val enableBit = RegInit(false.B)
  enableBit := io.csr_pf_enable

  val p0_fire, p1_fire, p2_fire           = WireInit(false.B)
  val p0_discard, p1_discard, p2_discard  = WireInit(false.B)
  val p0_ready, p1_ready, p2_ready        = WireInit(false.B)

  /**
    ******************************************************************************
    * IPrefetch Stage 0
    * - 1. send req to IMeta
    * - 2. send req to ITLB (no blocked)
    * - 3. check last req
    ******************************************************************************
    */
  val p0_valid  = fromFtq.req.valid

  val p0_vaddr      = fromFtq.req.bits.target
  val p0_req_cancel = Wire(Bool())

  /** 1. send req to IMeta */
  toIMeta.valid     := p0_valid && !p0_req_cancel
  toIMeta.bits.idx  := get_idx(p0_vaddr)

  /** 2. send req to ITLB (no blocked) */
  toITLB.valid                    := p0_valid && !p0_req_cancel
  toITLB.bits.size                := 3.U // TODO: fix the size
  toITLB.bits.vaddr               := p0_vaddr
  toITLB.bits.debug.pc            := p0_vaddr
  toITLB.bits.kill                := DontCare
  toITLB.bits.cmd                 := TlbCmd.exec
  toITLB.bits.debug.robIdx        := DontCare
  toITLB.bits.debug.isFirstIssue  := DontCare
  toITLB.bits.memidx              := DontCare
  toITLB.bits.no_translate        := false.B
  fromITLB.ready                  := true.B
  // TODO: whether to handle tlb miss for prefetch
  io.iTLBInter.req_kill           := true.B

  /** FTQ request port */
  fromFtq.req.ready := p0_ready

  /** stage 0 control */
  // Cancel request when prefetch not enable
  p0_req_cancel := !enableBit
  p0_ready      := p1_ready && toITLB.ready && toIMeta.ready && !io.flush
  p0_fire       := p0_valid && p0_ready && !p0_req_cancel && !io.flush
  p0_discard    := p0_valid && p0_req_cancel

  /**
    ******************************************************************************
    * IPrefetch Stage 1
    * - 1. Receive resp from ITLB (no blocked)
    * - 2. Receive resp from IMeta and check
    * - 3. Monitor the requests from missUnit to write to SRAM.
    ******************************************************************************
    */
  val p1_valid  = generatePipeControl(lastFire = p0_fire, thisFire = p1_fire || p1_discard, thisFlush = io.flush, lastFlush = false.B)

  val p1_vsetIdx = RegEnable(get_idx(p0_vaddr), p0_fire)
  val p1_req_cancel = Wire(Bool())

  /** 1. Receive resp from ITLB (no blocked) */
  val p1_tlb_miss   = ResultHoldBypass(valid = RegNext(p0_fire), data = fromITLB.bits.miss)
  val p1_tlb_except = ResultHoldBypass(valid = RegNext(p0_fire), data = fromITLB.bits.excp(0).pf.instr || fromITLB.bits.excp(0).af.instr)
  val p1_paddr      = ResultHoldBypass(valid = RegNext(p0_fire), data = fromITLB.bits.paddr(0))

  /** 2. Receive resp from IMeta. */
  val meta_hit    = (fromIMeta zip fromIMetaValid).map{ case(way, valid) => (way.tag === get_phy_tag(p1_paddr)) && valid}.reduce(_||_)
  val p1_meta_hit = ResultHoldBypass(valid = RegNext(p0_fire), data = meta_hit)

  /** 3. Monitor the requests from missUnit to write to SRAM */
  // Continuous checking with combinatorial logic.
  val p1_monitor_hit = (fromMSHR.bits.blkPaddr === getBlkAddr(p1_paddr)) &&
                       (fromMSHR.bits.vsetIdx === p1_vsetIdx) && fromMSHR.valid

  /** Stage 1 control */
  p1_req_cancel := p1_tlb_miss || p1_tlb_except || p1_meta_hit || p1_monitor_hit
  p1_ready      := p1_valid && p2_ready || !p1_valid
  p1_fire       := p1_valid && p2_ready && !p1_req_cancel && !io.flush
  p1_discard    := p1_valid && p1_req_cancel

  /**
    ******************************************************************************
    * IPrefetch Stage 2
    * - 1. PMP Check: send req and receive resq in the same cycle
    * - 2. send req to missUnit
    ******************************************************************************
    */
  val p2_valid = generatePipeControl(lastFire = p1_fire, thisFire = p2_fire || p2_discard, thisFlush = io.flush, lastFlush = false.B)

  val p2_paddr      = RegEnable(p1_paddr, p1_fire)
  val p2_vsetIdx    = RegEnable(p1_vsetIdx, p1_fire)
  val p2_req_cancel = Wire(Bool())

  /** 1. PMP Check */
  val p2_pmp_except  = DataHoldBypass((fromPMP.mmio && !fromPMP.instr) || fromPMP.instr, RegNext(p1_fire))
  // PMP request
  toPMP.valid     := p2_valid
  toPMP.bits.addr := p2_paddr
  toPMP.bits.size := 3.U
  toPMP.bits.cmd  := TlbCmd.exec

  // /** 3. check mainPipe s2 */
  // val p2_mainPipe_hit = io.mainPipeInfo.s2.map(info => info.valid && (info.bits === getBlkAddr(p2_paddr))).reduce(_||_)

  /** Stage 2 control */
  p2_req_cancel := p2_pmp_except || io.prefetch_req_hit
  p2_ready      := p2_valid && toMSHR.ready || !p2_valid
  p2_fire       := toMSHR.fire
  p2_discard    := p2_valid && p2_req_cancel

  /** 4. send req to missUnit */
  toMSHR.valid          := p2_valid && !p2_req_cancel && !io.flush
  toMSHR.bits.blkPaddr  := getBlkAddr(p2_paddr)
  toMSHR.bits.vsetIdx   := p2_vsetIdx

  /** PerfAccumulate */
  // the number of prefetch request received from ftq
  XSPerfAccumulate("prefetch_req_receive", fromFtq.req.fire)
  // the number of prefetch request sent to missUnit
  XSPerfAccumulate("prefetch_req_send", toMSHR.fire)
  /**
    * Count the number of requests that are filtered for various reasons.
    * The number of prefetch discard in Performance Accumulator may be
    * a littel larger the number of really discarded. Because there can
    * be multiple reasons for a canceled request at the same time.
    */
  // discard prefetch request by flush
  XSPerfAccumulate("fdip_prefetch_discard_by_fencei",      p0_discard && io.flush)
  // discard prefetch request by tlb miss
  XSPerfAccumulate("fdip_prefetch_discard_by_tlb_miss",    p1_discard && p1_tlb_miss)
  // discard prefetch request by tlb except
  XSPerfAccumulate("fdip_prefetch_discard_by_tlb_except",  p1_discard && p1_tlb_except)
  // discard prefetch request by hit icache SRAM
  XSPerfAccumulate("fdip_prefetch_discard_by_hit_cache",   p2_discard && p1_meta_hit)
  // discard prefetch request by hit wirte SRAM
  XSPerfAccumulate("fdip_prefetch_discard_by_p1_monoitor", p1_discard && p1_monitor_hit)
  // discard prefetch request by pmp except or mmio
  XSPerfAccumulate("fdip_prefetch_discard_by_pmp",         p2_discard && p2_pmp_except)
  // discard prefetch request by hit mainPipe info
  // XSPerfAccumulate("fdip_prefetch_discard_by_mainPipe",    p2_discard && p2_mainPipe_hit)
} 