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

package xiangshan.frontend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import chisel3.experimental.chiselName
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}
import firrtl.stage.RunFirrtlTransformAnnotation
import firrtl.transforms.RenameModules
import freechips.rocketchip.transforms.naming.RenameDesiredNames

import scala.math.min
import scala.util.matching.Regex
import os.followLink

trait TageParams extends HasBPUConst with HasXSParameter {
  // println(BankTageTableInfos)
  val BankTageNTables = BankTageTableInfos.map(_.size) // Number of tage tables
  val UBitPeriod = 256
  val TageCtrBits = 3
  val uFoldedWidth = 32
  val TickWidth = 8



  val TotalBits = BankTageTableInfos.map { info =>
    info.map{
      case (s, h, t) => {
        s * (1+t+TageCtrBits)
      }
    }.reduce(_+_)
  }.reduce(_+_)

}

trait HasFoldedHistory {
  val histLen: Int
  val phistLen: Int
  def compute_folded_hist(hist: UInt, l: Int)(histLen: Int) = {
    if (histLen > 0) {
      val nChunks = (histLen + l - 1) / l
      val hist_chunks = (0 until nChunks) map {i =>
        hist(min((i+1)*l, histLen)-1, i*l)
      }
      ParallelXOR(hist_chunks)
    }
    else 0.U
  }
  val compute_folded_ghist = compute_folded_hist(_: UInt, _: Int)(histLen)
  val compute_folded_phist = compute_folded_hist(_: UInt, _: Int)(phistLen)
}

abstract class TageBundle(implicit p: Parameters)
  extends XSBundle with TageParams with BPUUtils

abstract class TageModule(implicit p: Parameters)
  extends XSModule with TageParams with BPUUtils
  {}


class TageReq(implicit p: Parameters) extends TageBundle {
  val pc = UInt(VAddrBits.W)
  val folded_hist = new AllFoldedHistories(foldedGHistInfos)
  val phist = UInt(PathHistoryLength.W)
}

class TageResp(implicit p: Parameters) extends TageBundle {
  val ctr = UInt(TageCtrBits.W)
  val u = Bool()
}

class TageUpdate(implicit p: Parameters) extends TageBundle {
  val pc = UInt(VAddrBits.W)
  val folded_hist = new AllFoldedHistories(foldedGHistInfos)
  val phist = UInt(PathHistoryLength.W)
  // update tag and ctr
  val mask = Bool()
  val taken = Bool()
  val alloc = Bool()
  val oldCtr = UInt(TageCtrBits.W)
  // update u
  val uMask = Bool()
  val u = UInt(2.W)
  val reset_u = Bool()
}

class TageMeta(val bank: Int)(implicit p: Parameters)
  extends TageBundle with HasSCParameter
{
  val provider = ValidUndirectioned(UInt(log2Ceil(BankTageNTables(bank)).W))
  val prednum = ValidUndirectioned(UInt(log2Ceil(BankTageNTables(bank)).W))
  val altprednum = ValidUndirectioned(UInt(log2Ceil(BankTageNTables(bank)).W))
  val altDiffers = Bool()
  val providerU = Bool()
  val providerCtr = UInt(TageCtrBits.W)
  val basecnt = UInt(2.W)
  val predcnt = UInt(3.W)
  val altpredhit = Bool()
  val altpredcnt = UInt(3.W)
  val allocate = ValidUndirectioned(UInt(log2Ceil(BankTageNTables(bank)).W))
  val taken = Bool()
  val scMeta = new SCMeta(EnableSC, BankSCNTables(bank))
  val pred_cycle = if (!env.FPGAPlatform) Some(UInt(64.W)) else None
}

class FakeTageTable()(implicit p: Parameters) extends TageModule {
  val io = IO(new Bundle() {
    val req = Input(Valid(new TageReq))
    val resp = Output(Vec(TageBanks, Valid(new TageResp)))
    val update = Input(new TageUpdate)
  })
  io.resp := DontCare

}

trait TBTParams extends HasXSParameter {
  val BtSize = 2048
  val bypassEntries = 4
}

@chiselName
class TageBTable(implicit p: Parameters) extends XSModule with TBTParams{
  val io = IO(new Bundle {
    val s0_fire = Input(Bool())
    val s0_pc   = Input(UInt(VAddrBits.W))
    val s1_cnt     = Output(Vec(numBr,UInt(2.W)))
    val update_cnt  = Input(Vec(numBr,UInt(2.W)))
   // val update  = Input(new TageUpdate)
    val update = Flipped(Valid(new BranchPredictionUpdate))
  })

  val bimAddr = new TableAddr(log2Up(BtSize), 1)

  val bt = Module(new SRAMTemplate(UInt(2.W), set = BtSize, way=numBr, shouldReset = false, holdRead = true))

  val doing_reset = RegInit(true.B)
  val resetRow = RegInit(0.U(log2Ceil(BtSize).W))
  resetRow := resetRow + doing_reset
  when (resetRow === (BtSize-1).U) { doing_reset := false.B }

  val s0_idx = bimAddr.getIdx(io.s0_pc)
  bt.io.r.req.valid := io.s0_fire
  bt.io.r.req.bits.setIdx := s0_idx

  val s1_read = bt.io.r.resp.data

  io.s1_cnt := bt.io.r.resp.data

  // Update logic
  val u_valid = io.update.valid
  val update = io.update.bits

  val u_idx = bimAddr.getIdx(update.pc)
  val need_to_update = VecInit((0 until numBr).map(i => u_valid && update.ftb_entry.brValids(i)/* && update_mask(i)*/))

  val newCtrs = Wire(Vec(numBr, UInt(2.W)))

  val wrbypass = Module(new WrBypass(UInt(2.W), bypassEntries, log2Up(BtSize), numWays = numBr))
  wrbypass.io.wen := need_to_update.reduce(_||_)
  wrbypass.io.write_idx := u_idx
  wrbypass.io.write_data := newCtrs
  wrbypass.io.write_way_mask.map(_ := need_to_update)


  val oldCtrs =
    VecInit((0 until numBr).map(i =>
      Mux(wrbypass.io.hit && wrbypass.io.hit_data(i).valid,
        wrbypass.io.hit_data(i).bits,
        io.update_cnt(i))
    ))

  def satUpdate(old: UInt, len: Int, taken: Bool): UInt = {
    val oldSatTaken = old === ((1 << len)-1).U
    val oldSatNotTaken = old === 0.U
    Mux(oldSatTaken && taken, ((1 << len)-1).U,
      Mux(oldSatNotTaken && !taken, 0.U,
        Mux(taken, old + 1.U, old - 1.U)))
  }

  val newTakens = update.preds.br_taken_mask
  newCtrs := VecInit((0 until numBr).map(i =>
    satUpdate(oldCtrs(i), 2, newTakens(i))
  ))

  bt.io.w.apply(
    valid = need_to_update.asUInt.orR || doing_reset,
    data = Mux(doing_reset, VecInit(Seq.fill(numBr)(2.U(2.W))), newCtrs),
    setIdx = Mux(doing_reset, resetRow, u_idx),
    waymask = Mux(doing_reset, Fill(numBr, 1.U(1.W)).asUInt(), need_to_update.asUInt())
  )

}


@chiselName
class TageTable
(
  val nRows: Int, val histLen: Int, val tagLen: Int, val uBitPeriod: Int, val tableIdx: Int
)(implicit p: Parameters)
  extends TageModule with HasFoldedHistory {
  val io = IO(new Bundle() {
    val req = Flipped(DecoupledIO(new TageReq))
    val resp = Output(Valid(new TageResp))
    val update = Input(new TageUpdate)
  })

  
  val SRAM_SIZE = 128 // physical size
  require(nRows % SRAM_SIZE == 0)
  val nBanks = 4
  val bankSize = nRows / nBanks
  val bankFoldWidth = if (bankSize >= SRAM_SIZE) bankSize / SRAM_SIZE else 1

  if (bankSize < SRAM_SIZE) {
    println(f"warning: tage table $tableIdx has small sram depth of $bankSize")
  }
  val bankIdxWidth = log2Ceil(nBanks)
  def get_bank_mask(idx: UInt) = VecInit((0 until nBanks).map(idx(bankIdxWidth-1, 0) === _.U))
  def get_bank_idx(idx: UInt) = idx >> bankIdxWidth

  // bypass entries for tage update
  val wrBypassEntries = 16
  val phistLen = if (PathHistoryLength > histLen) histLen else PathHistoryLength


  val idxFhInfo = (histLen, min(log2Ceil(nRows), histLen))
  val tagFhInfo = (histLen, min(histLen, tagLen))
  val altTagFhInfo = (histLen, min(histLen, tagLen-1))
  val allFhInfos = Seq(idxFhInfo, tagFhInfo, altTagFhInfo)

  def getFoldedHistoryInfo = allFhInfos.filter(_._1 >0).toSet
  def compute_tag_and_hash(unhashed_idx: UInt, allFh: AllFoldedHistories) = {
    val idx_fh = allFh.getHistWithInfo(idxFhInfo).folded_hist
    val tag_fh = allFh.getHistWithInfo(tagFhInfo).folded_hist
    val alt_tag_fh = allFh.getHistWithInfo(altTagFhInfo).folded_hist
    // require(idx_fh.getWidth == log2Ceil(nRows))
    val idx = (unhashed_idx ^ idx_fh)(log2Ceil(nRows)-1, 0)
    val tag = ((unhashed_idx >> log2Ceil(nRows)) ^ tag_fh ^ (alt_tag_fh << 1)) (tagLen - 1, 0)
    (idx, tag)
  }

  def inc_ctr(ctr: UInt, taken: Bool): UInt = satUpdate(ctr, TageCtrBits, taken)

  class TageEntry() extends TageBundle {
    // val valid = Bool()
    val tag = UInt(tagLen.W)
    val ctr = UInt(TageCtrBits.W)
  }

  val validArray = RegInit(0.U(nRows.W))


  // pc is start address of basic block, most 2 branch inst in block
  // def getUnhashedIdx(pc: UInt) = pc >> (instOffsetBits+log2Ceil(TageBanks))
  def getUnhashedIdx(pc: UInt): UInt = pc >> instOffsetBits

  // val s1_pc = io.req.bits.pc
  val req_unhashed_idx = getUnhashedIdx(io.req.bits.pc)

  val us = Module(new Folded1WDataModuleTemplate(Bool(), nRows, 1, isSync=true, width=uFoldedWidth))

  val table_banks = Seq.fill(nBanks)(
    Module(new FoldedSRAMTemplate(new TageEntry, set=nRows/nBanks, width=bankFoldWidth, shouldReset=false, holdRead=true, singlePort=true)))
  // val table  = Module(new SRAMTemplate(new TageEntry, set=nRows, way=1, shouldReset=false, holdRead=true, singlePort=false))


  val (s0_idx, s0_tag) = compute_tag_and_hash(req_unhashed_idx, io.req.bits.folded_hist)
  val s0_bank_req_1h = get_bank_mask(s0_idx)

  for (b <- 0 until nBanks) {
    table_banks(b).io.r.req.valid := io.req.fire && s0_bank_req_1h(b)
    table_banks(b).io.r.req.bits.setIdx := get_bank_idx(s0_idx)
  }

  us.io.raddr(0) := s0_idx
  // us.io.raddr(1) := DontCare
  // us.io.raddr(2) := DontCare


  val s1_idx = RegEnable(s0_idx, io.req.fire)
  val s1_tag = RegEnable(s0_tag, io.req.fire)
  val s1_bank_req_1h = RegEnable(s0_bank_req_1h, io.req.fire)

  val tables_r = table_banks.map(_.io.r.resp.data(0)) // s1

  val resp_selected = Mux1H(s1_bank_req_1h, tables_r)
  val req_rhit = validArray(s1_idx) && resp_selected.tag === s1_tag

  io.resp.valid := req_rhit
  io.resp.bits.ctr := resp_selected.ctr
  io.resp.bits.u := us.io.rdata(0)

  // reset all us in 32 cycles
  us.io.resetEn.map(_ := io.update.reset_u)


  // Use fetchpc to compute hash
  val update_wdata = Wire(new TageEntry)

  // val (update_idx, update_tag) =  compute_tag_and_hash(getUnhashedIdx(io.update.pc), io.update.hist, io.update.phist)
  val (update_idx, update_tag) =  compute_tag_and_hash(getUnhashedIdx(io.update.pc), io.update.folded_hist)
  val update_req_bank_1h = get_bank_mask(update_idx)
  val update_idx_in_bank = get_bank_idx(update_idx)

  val not_silent_update = Wire(Bool())

  for (b <- 0 until nBanks) {
    table_banks(b).io.w.apply(
      valid   = io.update.mask && update_req_bank_1h(b) && not_silent_update/*  && !s0_bank_req_1h(b) */,
      data    = update_wdata,
      setIdx  = update_idx_in_bank,
      waymask = true.B
    )
  }

  // val writeBuffers = Seq.fill(nBanks)(Queue())

  val bank_conflict = (0 until nBanks).map(b => table_banks(b).io.w.req.valid && s0_bank_req_1h(b)).reduce(_||_)
  io.req.ready := true.B
  // io.req.ready := !(io.update.mask && not_silent_update)
  // io.req.ready := !bank_conflict
  XSPerfAccumulate(f"tage_table_bank_conflict", bank_conflict)

  val newValidArray = VecInit(validArray.asBools)
  when (io.update.mask) {
    newValidArray(update_idx) := true.B
    validArray := newValidArray.asUInt
  }

  us.io.wen := io.update.uMask
  us.io.waddr := update_idx
  us.io.wdata := io.update.u


  val wrbypass = Module(new WrBypass(UInt(TageCtrBits.W), wrBypassEntries, log2Ceil(nRows), tagWidth=tagLen))

  wrbypass.io.wen := io.update.mask && not_silent_update
  wrbypass.io.write_data.map(_ := update_wdata.ctr)

  val bypass_ctr = wrbypass.io.hit_data(0).bits
  update_wdata.ctr   := Mux(io.update.alloc,
    Mux(io.update.taken, 4.U, 3.U),
    Mux(wrbypass.io.hit,
          inc_ctr(bypass_ctr, io.update.taken),
          inc_ctr(io.update.oldCtr, io.update.taken)
    )
  )
  update_wdata.tag   := update_tag

  // remove silent updates
  def silentUpdate(ctr: UInt, taken: Bool) = {
    ctr.andR && taken || !ctr.orR && !taken
  }
  not_silent_update :=
    Mux(wrbypass.io.hit,
      !silentUpdate(bypass_ctr, io.update.taken),
      !silentUpdate(io.update.oldCtr, io.update.taken)) ||
    io.update.alloc

  wrbypass.io.write_idx := update_idx
  wrbypass.io.write_tag.map(_ := update_tag)



  XSPerfAccumulate(f"tage_table_wrbypass_hit", io.update.mask && wrbypass.io.hit)
  XSPerfAccumulate(f"tage_table_wrbypass_enq", io.update.mask && !wrbypass.io.hit)

  XSPerfAccumulate(f"tage_table_real_updates", io.update.mask && not_silent_update)
  XSPerfAccumulate(f"tage_table_silent_updates_eliminated", io.update.mask && !not_silent_update)

  XSPerfAccumulate("tage_table_hits", PopCount(io.resp.valid))

  val u = io.update
  val b = PriorityEncoder(u.mask)
  val ub = PriorityEncoder(u.uMask)
  XSDebug(io.req.fire,
    p"tableReq: pc=0x${Hexadecimal(io.req.bits.pc)}, " +
    p"idx=$s0_idx, tag=$s0_tag\n")
  XSDebug(RegNext(io.req.fire) && req_rhit,
    p"TageTableResp: idx=$s1_idx, hit:$req_rhit, " +
    p"ctr:${io.resp.bits.ctr}, u:${io.resp.bits.u}\n")
  XSDebug(io.update.mask,
    p"update Table: pc:${Hexadecimal(u.pc)}}, " +
    p"taken:${u.taken}, alloc:${u.alloc}, oldCtr:${u.oldCtr}\n")
  XSDebug(io.update.mask,
    p"update Table: writing tag:$update_tag, " +
    p"ctr: ${update_wdata.ctr} in idx ${update_idx}\n")
  XSDebug(RegNext(io.req.fire) && !req_rhit, "TageTableResp: not hit!\n")

  // ------------------------------Debug-------------------------------------
  val valids = Reg(Vec(nRows, Bool()))
  when (reset.asBool) { valids.foreach(r => r := false.B) }
  when (io.update.mask) { valids(update_idx) := true.B }
  XSDebug("Table usage:------------------------\n")
  XSDebug("%d out of %d rows are valid\n", PopCount(valids), nRows.U)

}

abstract class BaseTage(implicit p: Parameters) extends BasePredictor with TageParams with BPUUtils {
}

class FakeTage(implicit p: Parameters) extends BaseTage {
  io.out <> 0.U.asTypeOf(DecoupledIO(new BasePredictorOutput))

  // io.s0_ready := true.B
  io.s1_ready := true.B
  io.s2_ready := true.B
}

@chiselName
class Tage(implicit p: Parameters) extends BaseTage {

  val resp_meta = Wire(MixedVec((0 until TageBanks).map(new TageMeta(_))))
  override val meta_size = resp_meta.getWidth
  val bank_tables = BankTageTableInfos.zipWithIndex.map {
    case (info, b) =>
      val tables = info.zipWithIndex.map {
        case ((nRows, histLen, tagLen), i) =>
          val t = Module(new TageTable(nRows, histLen, tagLen, UBitPeriod, i))
          t.io.req.valid := io.s0_fire
          t.io.req.bits.pc := s0_pc
          t.io.req.bits.folded_hist := io.in.bits.folded_hist
          t.io.req.bits.phist := io.in.bits.phist
          t
      }
      tables
  }
  val bt = Module (new TageBTable)
  bt.io.s0_fire := io.s0_fire
  bt.io.s0_pc   := s0_pc
  bt.io.update := io.update

  val bankTickCtrs = Seq.fill(BankTageTableInfos.length)(RegInit(0.U(TickWidth.W)))

  val tage_fh_info = bank_tables.flatMap(_.map(_.getFoldedHistoryInfo).reduce(_++_)).toSet
  override def getFoldedHistoryInfo = Some(tage_fh_info)

  val s1_resps = MixedVecInit(bank_tables.map(b => VecInit(b.map(t => t.io.resp))))

  //val s1_bim = io.in.bits.resp_in(0).s1.preds
  // val s2_bim = RegEnable(s1_bim, enable=io.s1_fire)

  val debug_pc_s0 = s0_pc
  val debug_pc_s1 = RegEnable(s0_pc, enable=io.s0_fire)
  val debug_pc_s2 = RegEnable(debug_pc_s1, enable=io.s1_fire)

  val s1_tageTakens    = Wire(Vec(TageBanks, Bool()))
  val s1_provideds     = Wire(Vec(TageBanks, Bool()))
  val s1_providers     = Wire(MixedVec(BankTageNTables.map(n=>UInt(log2Ceil(n).W))))
  val s1_finalAltPreds = Wire(Vec(TageBanks, Bool()))
  val s1_providerUs    = Wire(Vec(TageBanks, Bool()))
  val s1_providerCtrs  = Wire(Vec(TageBanks, UInt(TageCtrBits.W)))
  val s1_prednums      = Wire(MixedVec(BankTageNTables.map(n=>UInt(log2Ceil(n).W))))
  val s1_altprednums   = Wire(MixedVec(BankTageNTables.map(n=>UInt(log2Ceil(n).W))))
  val s1_predcnts      = Wire(Vec(TageBanks, UInt(TageCtrBits.W)))
  val s1_altpredcnts   = Wire(Vec(TageBanks, UInt(TageCtrBits.W)))
  val s1_altpredhits   = Wire(Vec(TageBanks, Bool()))
  val s1_basecnts      = Wire(Vec(TageBanks, UInt(2.W)))

  val s2_tageTakens    = RegEnable(s1_tageTakens, io.s1_fire)
  val s2_provideds     = RegEnable(s1_provideds, io.s1_fire)
  val s2_providers     = RegEnable(s1_providers, io.s1_fire)
  val s2_finalAltPreds = RegEnable(s1_finalAltPreds, io.s1_fire)
  val s2_providerUs    = RegEnable(s1_providerUs, io.s1_fire)
  val s2_providerCtrs  = RegEnable(s1_providerCtrs, io.s1_fire)
  val s2_prednums      = RegEnable(s1_prednums, io.s1_fire)
  val s2_altprednums   = RegEnable(s1_altprednums, io.s1_fire)
  val s2_predcnts      = RegEnable(s1_predcnts, io.s1_fire)
  val s2_altpredcnts   = RegEnable(s1_altpredcnts, io.s1_fire)
  val s2_altpredhits   = RegEnable(s1_altpredhits, io.s1_fire)
  val s2_basecnts      = RegEnable(s1_basecnts, io.s1_fire)

  io.out.resp := io.in.bits.resp_in(0)
  io.out.last_stage_meta := resp_meta.asUInt

  // val ftb_hit = io.in.bits.resp_in(0).s2.preds.hit
  val ftb_entry = io.in.bits.resp_in(0).s2.ftb_entry
  val resp_s2 = io.out.resp.s2

  // Update logic
  val u_valid = io.update.valid
  val update = io.update.bits
  val updateValids = VecInit((0 until TageBanks).map(w =>
      update.ftb_entry.brValids(w) && u_valid && !update.ftb_entry.always_taken(w) &&
      !(PriorityEncoder(update.preds.br_taken_mask) < w.U)))
  val updatePhist = update.phist
  val updateFHist = update.folded_hist

  val updateMetas = update.meta.asTypeOf(MixedVec((0 until TageBanks).map(new TageMeta(_))))

  val updateMask    = WireInit(0.U.asTypeOf(MixedVec(BankTageNTables.map(Vec(_, Bool())))))
  val updateUMask   = WireInit(0.U.asTypeOf(MixedVec(BankTageNTables.map(Vec(_, Bool())))))
  val updateResetU  = WireInit(0.U.asTypeOf(MixedVec(BankTageNTables.map(_=>Bool())))) // per predictor
  val updateTaken   = Wire(MixedVec(BankTageNTables.map(Vec(_, Bool()))))
  val updateAlloc   = Wire(MixedVec(BankTageNTables.map(Vec(_, Bool()))))
  val updateOldCtr  = Wire(MixedVec(BankTageNTables.map(Vec(_, UInt(TageCtrBits.W)))))
  val updateU       = Wire(MixedVec(BankTageNTables.map(Vec(_, Bool()))))
  val updatebcnt    = Wire(Vec(TageBanks, UInt(2.W)))
  val baseupdate    = Wire(Vec(TageBanks,Bool()))
  updateTaken   := DontCare
  updateAlloc   := DontCare
  updateOldCtr  := DontCare
  updateU       := DontCare

  val updateMisPreds = update.mispred_mask

  // access tag tables and output meta info
  for (w <- 0 until TageBanks) {
    val s1_tageTaken     = WireInit(bt.io.s1_cnt(w)(1))
    var s1_altPred       = WireInit(bt.io.s1_cnt(w)(1))
    val s1_finalAltPred  = WireInit(bt.io.s1_cnt(w)(1))
    var s1_provided      = false.B
    var s1_provider      = 0.U
    var s1_altprednum    = 0.U
    var s1_altpredhit    = false.B
    var s1_prednum       = 0.U
    var s1_basecnt       = 0.U

    for (i <- 0 until BankTageNTables(w)) {
      val hit = s1_resps(w)(i).valid
      val ctr = s1_resps(w)(i).bits.ctr
      when (hit) {
        s1_tageTaken := Mux(ctr === 3.U || ctr === 4.U, s1_altPred, ctr(2)) // Use altpred on weak taken
        s1_finalAltPred := s1_altPred
      }
      s1_altpredhit = (s1_provided && hit) || s1_altpredhit        // Once hit then provide
      s1_provided = s1_provided || hit          // Once hit then provide
      s1_provider = Mux(hit, i.U, s1_provider)  // Use the last hit as provider
      s1_altPred = Mux(hit, ctr(2), s1_altPred) // Save current pred as potential altpred
      s1_altprednum = Mux(hit,s1_prednum,s1_altprednum)      // get altpredict table number
      s1_prednum = Mux(hit,i.U,s1_prednum)      // get predict table number
    }
    s1_provideds(w)      := s1_provided
    s1_basecnts(w)       := bt.io.s1_cnt(w)
    s1_providers(w)      := s1_provider
    s1_finalAltPreds(w)  := s1_finalAltPred
    s1_tageTakens(w)     := s1_tageTaken
    s1_providerUs(w)     := s1_resps(w)(s1_provider).bits.u
    s1_providerCtrs(w)   := s1_resps(w)(s1_provider).bits.ctr
    s1_prednums(w)       := s1_prednum
    s1_altprednums(w)    := s1_altprednum
    s1_predcnts(w)       := s1_resps(w)(s1_prednum).bits.ctr
    s1_altpredhits(w)    := s1_altpredhit
    s1_altpredcnts(w)    := s1_resps(w)(s1_altprednum).bits.ctr

    resp_meta(w).provider.valid   := s2_provideds(w)
    resp_meta(w).provider.bits    := s2_providers(w)
    resp_meta(w).prednum.valid    := s2_provideds(w)
    resp_meta(w).prednum.bits     := s2_prednums(w)
    resp_meta(w).altprednum.valid := s2_altpredhits(w)
    resp_meta(w).altprednum.bits  := s2_altprednums(w)
    resp_meta(w).altDiffers       := s2_finalAltPreds(w) =/= s2_tageTakens(w)
    resp_meta(w).providerU        := s2_providerUs(w)
    resp_meta(w).providerCtr      := s2_providerCtrs(w)
    resp_meta(w).predcnt          := s2_predcnts(w)
    resp_meta(w).altpredcnt       := s2_altpredcnts(w)
    resp_meta(w).altpredhit       := s2_altpredhits(w)
    resp_meta(w).taken            := s2_tageTakens(w)
    resp_meta(w).basecnt          := s2_basecnts(w)
    resp_meta(w).pred_cycle.map(_ := GTimer())

    // Create a mask fo tables which did not hit our query, and also contain useless entries
    // and also uses a longer history than the provider
    val allocatableSlots =
      RegEnable(
        VecInit(s1_resps(w).map(r => !r.valid && !r.bits.u)).asUInt &
          ~(LowerMask(UIntToOH(s1_provider), BankTageNTables(w)) &
            Fill(BankTageNTables(w), s1_provided.asUInt)),
        io.s1_fire
      )
    val allocLFSR   = LFSR64()(BankTageNTables(w) - 1, 0)
    val firstEntry  = PriorityEncoder(allocatableSlots)
    val maskedEntry = PriorityEncoder(allocatableSlots & allocLFSR)
    val allocEntry  = Mux(allocatableSlots(maskedEntry), maskedEntry, firstEntry)
    resp_meta(w).allocate.valid := allocatableSlots =/= 0.U
    resp_meta(w).allocate.bits  := allocEntry

    // Update in loop
    val updateValid = updateValids(w)
    val updateMeta = updateMetas(w)
    val isUpdateTaken = updateValid && update.preds.br_taken_mask(w)
    val updateMisPred = updateMisPreds(w)
    val up_altpredhit = updateMeta.altpredhit
    val up_prednum    = updateMeta.prednum.bits
    val up_altprednum = updateMeta.altprednum.bits
    when (updateValid) {
      when (updateMeta.provider.valid) {
        when (updateMisPred && up_altpredhit && (updateMeta.predcnt === 3.U || updateMeta.predcnt === 4.U)){
        updateMask(w)(up_altprednum)   := true.B
        updateUMask(w)(up_altprednum)  := false.B
        updateTaken(w)(up_altprednum)  := isUpdateTaken
        updateOldCtr(w)(up_altprednum) := updateMeta.altpredcnt
        updateAlloc(w)(up_altprednum)  := false.B

        }
        updateMask(w)(up_prednum)   := true.B
        updateUMask(w)(up_prednum)  := true.B

        updateU(w)(up_prednum) := Mux(!updateMeta.altDiffers, updateMeta.providerU, !updateMisPred)
        updateTaken(w)(up_prednum)  := isUpdateTaken
        updateOldCtr(w)(up_prednum) := updateMeta.predcnt
        updateAlloc(w)(up_prednum)  := false.B
      }
    }

    // update base table if used base table to predict
    when (updateValid) {
      when(updateMeta.provider.valid) {
        when(~up_altpredhit && updateMisPred && (updateMeta.predcnt === 3.U || updateMeta.predcnt === 4.U)) {
        baseupdate(w) := true.B
        }
        .otherwise{
          baseupdate(w) := false.B
        }
      }
      .otherwise{
        baseupdate(w) := true.B
      }
    }
    .otherwise{
      baseupdate(w) := false.B
    }
    updatebcnt(w) := updateMeta.basecnt

    // if mispredicted and not the case that
    // provider offered correct target but used altpred due to unconfident
    when (updateValid && updateMisPred && ~((updateMeta.predcnt === 3.U && ~isUpdateTaken || updateMeta.predcnt === 4.U && isUpdateTaken) && updateMeta.provider.valid)) {
    //when (updateValid && updateMisPred) {
      val allocate = updateMeta.allocate
      bankTickCtrs(w) := satUpdate(bankTickCtrs(w), TickWidth, allocate.valid)
      when (allocate.valid) {
        updateMask(w)(allocate.bits)  := true.B
        updateTaken(w)(allocate.bits) := isUpdateTaken
        updateAlloc(w)(allocate.bits) := true.B
        updateUMask(w)(allocate.bits) := true.B
        updateU(w)(allocate.bits) := false.B
      }.otherwise {

        val provider = updateMeta.provider
        val decrMask = Mux(provider.valid, ~LowerMask(UIntToOH(provider.bits), BankTageNTables(w)), 0.U(BankTageNTables(w).W))
        for (i <- 0 until BankTageNTables(w)) {
          when (decrMask(i)) {
            updateUMask(w)(i) := true.B
            updateU(w)(i) := false.B
          }
        }
      }
    }
    when (bankTickCtrs(w) === ((1 << TickWidth) - 1).U) {
      bankTickCtrs(w) := 0.U
      updateResetU(w) := true.B
    }

    XSPerfAccumulate(s"tage_bank_${w}_reset_u", updateResetU(w))
    XSPerfAccumulate(s"tage_bank_${w}_mispred", updateValid && updateMisPred)
  }



  for (i <- 0 until numBr) {
    resp_s2.preds.br_taken_mask(i) := s2_tageTakens(i)
  }

  for (w <- 0 until TageBanks) {
    for (i <- 0 until BankTageNTables(w)) {
      bank_tables(w)(i).io.update.mask := RegNext(updateMask(w)(i))
      bank_tables(w)(i).io.update.taken := RegNext(updateTaken(w)(i))
      bank_tables(w)(i).io.update.alloc := RegNext(updateAlloc(w)(i))
      bank_tables(w)(i).io.update.oldCtr := RegNext(updateOldCtr(w)(i))

      bank_tables(w)(i).io.update.uMask := RegNext(updateUMask(w)(i))
      bank_tables(w)(i).io.update.u := RegNext(updateU(w)(i))
      bank_tables(w)(i).io.update.reset_u := RegNext(updateResetU(w))
      bank_tables(w)(i).io.update.pc := RegNext(update.pc)
      // use fetch pc instead of instruction pc
      bank_tables(w)(i).io.update.folded_hist := RegNext(updateFHist)
      bank_tables(w)(i).io.update.phist := RegNext(updatePhist)
    }
  }
  bt.io.update  := RegNext(io.update)
  bt.io.update.valid := RegNext(baseupdate.reduce(_||_))
  bt.io.update_cnt := RegNext(updatebcnt)

  // all should be ready for req
  io.s1_ready := bank_tables.flatMap(_.map(_.io.req.ready)).reduce(_&&_)
  XSPerfAccumulate(f"tage_write_blocks_read", !io.s1_ready)

  def pred_perf(name: String, cnt: UInt)   = XSPerfAccumulate(s"${name}_at_pred", cnt)
  def commit_perf(name: String, cnt: UInt) = XSPerfAccumulate(s"${name}_at_commit", cnt)
  def tage_perf(name: String, pred_cnt: UInt, commit_cnt: UInt) = {
    pred_perf(name, pred_cnt)
    commit_perf(name, commit_cnt)
  }

  // Debug and perf info
  for (b <- 0 until TageBanks) {
    for (i <- 0 until BankTageNTables(b)) {
      val pred_i_provided =
        s2_provideds(b) && s2_providers(b) === i.U
      val commit_i_provided =
        updateMetas(b).provider.valid && updateMetas(b).provider.bits === i.U && updateValids(b)
      tage_perf(
        s"bank_${b}_tage_table_${i}_provided",
        PopCount(pred_i_provided),
        PopCount(commit_i_provided)
      )
    }
    tage_perf(
      s"bank_${b}_tage_use_bim",
      PopCount(!s2_provideds(b)),
      PopCount(!updateMetas(b).provider.valid && updateValids(b))
    )
    def unconf(providerCtr: UInt) = providerCtr === 3.U || providerCtr === 4.U
    tage_perf(
      s"bank_${b}_tage_use_altpred",
      PopCount(s2_provideds(b) && unconf(s2_providerCtrs(b))),
      PopCount(updateMetas(b).provider.valid &&
        unconf(updateMetas(b).providerCtr) && updateValids(b))
    )
    tage_perf(
      s"bank_${b}_tage_provided",
      PopCount(s2_provideds(b)),
      PopCount(updateMetas(b).provider.valid && updateValids(b))
    )
  }

  for (b <- 0 until TageBanks) {
    val m = updateMetas(b)
    // val bri = u.metas(b)
    XSDebug(updateValids(b), "update(%d): pc=%x, cycle=%d, taken:%b, misPred:%d, bimctr:%d, pvdr(%d):%d, altDiff:%d, pvdrU:%d, pvdrCtr:%d, alloc(%d):%d\n",
      b.U, update.pc, 0.U, update.preds.br_taken_mask(b), update.mispred_mask(b),
      0.U, m.provider.valid, m.provider.bits, m.altDiffers, m.providerU, m.providerCtr, m.allocate.valid, m.allocate.bits
    )
  }
  val s2_resps = RegEnable(s1_resps, io.s1_fire)
  XSDebug("req: v=%d, pc=0x%x\n", io.s0_fire, s0_pc)
  XSDebug("s1_fire:%d, resp: pc=%x\n", io.s1_fire, debug_pc_s1)
  XSDebug("s2_fireOnLastCycle: resp: pc=%x, target=%x, hits=%b, takens=%b\n",
    debug_pc_s2, io.out.resp.s2.target, s2_provideds.asUInt, s2_tageTakens.asUInt)

  for (b <- 0 until TageBanks) {
    for (i <- 0 until BankTageNTables(b)) {
      XSDebug("bank(%d)_tage_table(%d): valid:%b, resp_ctr:%d, resp_us:%d\n",
        b.U, i.U, s2_resps(b)(i).valid, s2_resps(b)(i).bits.ctr, s2_resps(b)(i).bits.u)
    }
  }
    // XSDebug(io.update.valid && updateIsBr, p"update: sc: ${updateSCMeta}\n")
    // XSDebug(true.B, p"scThres: use(${useThreshold}), update(${updateThreshold})\n")
}


class Tage_SC(implicit p: Parameters) extends Tage with HasSC {}
