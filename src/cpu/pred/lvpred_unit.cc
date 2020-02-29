/*
 * Will have to ask what I'm supposed to put here lol
 */

#include "cpu/pred/basic_lvpred.hh"
#include "cpu/pred/lvpred_unit_impl.hh"
#include "cpu/pred/stride_hist.hh"
#include "cpu/pred/three_period.hh"

// #include "cpu/pred/hybrid.hh"
#include "cpu/pred/empty_lvp.hh"
#include "cpu/pred/fa3p.hh"

LVPredUnit *
LoadValuePredictorParams::create()
{
    if (lvpredType == "basic") {
        return new BasicLVP(this);
    } else if (lvpredType == "strideHist") {
        return new StrideHistLVP(this);
    } else if (lvpredType == "3period") {
        return new ThreePeriodLVP(this);
//     } else if (lvpredType == "hybrid") {
// 	return new HybridLVP(this);
    } else if (lvpredType == "fa3p") {
        return new FA3P(this);
    } else if (lvpredType == "none") {
        return new EmptyLVP(this);
    } else {
        fatal("Invalid LVP selected!");
    }
}
