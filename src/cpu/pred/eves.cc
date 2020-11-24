#include <inttypes.h>
// #include "cvp.h"


#include "cpu/pred/eves.hh"
#include <iostream>
//#include "cpu/pred/lvpred_unit.hh"


int seq_commit;
//typedef LoadValuePredictorParams Params;

#define NOTLLCMISS (actual_latency < 42)//150)
#define NOTL2MISS (actual_latency < 42)//60)
#define NOTL1MISS (actual_latency < 42)//12)
#define FASTINST (actual_latency <= 1)
#define MFASTINST (actual_latency < 3)  // shouldn't need to change these last two
// TODO: change these macros so they are consistent with gem5's Skylake delays.

// constructor
EvesLVP::EvesLVP(Params *params) : LVPredUnit(params) { lvpredType = "eves"; }  // CHECK: will this work?

bool isSlowALU (StaticInstPtr p) {
	// mul and div instructions are the only slow alu insts
	return (p->mnemonic[0] == 'm' && p->mnemonic[1] == 'u' && p->mnemonic[2] == 'l')
	  || (p->mnemonic[0] == 'd' && p->mnemonic[1] == 'i' && p->mnemonic[2] == 'v');
}

void
EvesLVP::getPredVtage (Addr pc, LVPredUnit::lvpReturnValues& U, uint64_t & predicted_value)
{
  bool predvtage = false;
  //uint64_t pc = pc;
  uint64_t PCindex = ((pc) ^ (pc >> 2) ^ (pc >> 5)) % PREDSIZE;
  uint64_t PCbank = (PCindex >> LOGBANK) << LOGBANK;
  for (int i = 1; i <= NHIST; i++)
    {
      U.GI[i] = (gi (i, pc) + (PCbank + (i << LOGBANK))) % PREDSIZE;
      U.GTAG[i] = gtag (i, pc);
    }
  U.GTAG[0] = (pc ^ (pc >> 4) ^ (pc >> TAGWIDTH)) & ((1 << TAGWIDTH) - 1);
  U.GI[0] = PCindex;
  U.HitBank = -1;
  // cout << "Setting U.HitBank to " << U.HitBank << " (-1)" << endl;

  for (int i = NHIST; i >= 0; i--)
    {
      if (Vtage[U.GI[i]].tag == U.GTAG[i])
	{
	  // cout << "Setting U.HitBank to i=" << i << endl;
	  U.HitBank = i;
	  break;
	}
    }

  if (LastMispVT >= 128)
// when a misprediction is encountered on VTAGE, we do not predict with VTAGE for 128 instructions;
// does not bring significant speed-up, but reduces the misprediction number significantly: mispredictions tend to be clustered       
    if (U.HitBank >= 0)
      {
	int index = Vtage[U.GI[U.HitBank]].hashpt;
	if (index < 3 * BANKDATA)
	  {
	    // the hash and the data are both present
	    U.predictedValue = LDATA[index].data;
	    predvtage = ((Vtage[U.GI[U.HitBank]].conf >= MAXCONFID));
	  }
      }
  U.predVtage = predvtage;
}

void
EvesLVP::getPredStride (Addr pc, LVPredUnit::lvpReturnValues& U, uint64_t & predicted_value, uint64_t seq_no)
{
  bool predstride = false;
  int B[NBWAYSTR];
  int TAG[NBWAYSTR];
  //uint64_t pc = pc;

//use a 3-way skewed-associative structure  for the stride predictor
  for (int i = 0; i < NBWAYSTR; i++)
    {
      //B[i] index in way i ; TAG[i] tag in way i;
      B[i] = ((((pc) ^ (pc >> (2 * LOGSTR - i)) ^
		(pc >> (LOGSTR - i)) ^
		(pc >> (3 * LOGSTR - i))) * NBWAYSTR) +
	      i) % (NBWAYSTR * (1 << LOGSTR));
      int j = (NBWAYSTR - i);
      if (j < 0)
	j = 0;

      TAG[i] =
	((pc >> (LOGSTR - j)) ^ (pc >> (2 * LOGSTR - j)) ^
	 (pc >> (3 * LOGSTR - j)) ^ (pc >> (4 * LOGSTR - j))) & ((1 <<
								  TAGWIDTHSTR)
								 - 1);
      U.B[i] = B[i];
      U.TAGSTR[i] = TAG[i];
    }

  int STHIT = -1;
  for (int i = 0; i < NBWAYSTR; i++)
    {
      if (STR[B[i]].tag == TAG[i])
	{
	  STHIT = B[i];
	  break;
	}
    }
  U.STHIT = STHIT;
  if (STHIT >= 0) {
    //std::cout << "STHIT >= 0... ";
    if (true) // (SafeStride >= 0)
      {				// hit
        uint64_t LastCommitedValue = STR[STHIT].LastValue;
        //std::cout << (uint64_t) STR[STHIT].conf << " >= " << MAXCONFIDSTR / 4 << "?"; 
	if (STR[STHIT].conf >= (MAXCONFIDSTR / 4))
	  {
	    int inflight = stored_inflight;
	    U.predictedValue =
	      (uint64_t) ((int64_t) LastCommitedValue +
			  ((inflight + 1) * ((int64_t) STR[STHIT].Stride)));
	    predstride = true;
	  }
      }  //std::cout << std::endl;
  }
  U.predStride = predstride;
  //if (predstride) std::cout << "Confident stride!" << std::endl;
}


//bool
LVPredUnit::lvpReturnValues
//getPrediction (uint64_t seq_no, uint64_t pc, uint8_t piece,
//	       uint64_t & predicted_value)
EvesLVP::makePrediction(TheISA::PCState pc, ThreadID tid, unsigned currentcycle)
{
	// TODO: function overloading review
	// small TODO: verify that predicted_value really isn't ever used
  LVPredUnit::lvpReturnValues U = LVPredUnit::lvpReturnValues();
  uint64_t predicted_value = 0;

  U.predVtage = false;
  U.predStride = false;

#ifndef PREDSTRIDE
//assert(false);
#endif

#ifdef PREDSTRIDE
  getPredStride (pc.pc(), U, predicted_value, stored_seq_no);
#endif
#ifdef PREDVTAGE
  getPredVtage (pc.pc(), U, predicted_value);
#endif
// the two predictions are very rarely both high confidence; when they are pick the VTAGE prediction
  U.confidence = -1;
  if (U.predStride || U.predVtage) U.confidence = 100;
  return U; 
}

// Update of the Stride predictor
// function determining whether to  update or not confidence on a correct prediction
bool
EvesLVP::strideupdateconf (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency,
		  int stride)
{
return true;
#define UPDATECONFSTR2 (((!U.prediction_result) || (U.predStride)) && ((random () & ((1 << (NOTLLCMISS + NOTL2MISS + NOTL1MISS + 2*MFASTINST  + 2*(!inst->isLoad()))) - 1)) == 0))
#define UPDATECONFSTR1 (abs (stride >= 8) ? (UPDATECONFSTR2 || UPDATECONFSTR2) : (UPDATECONFSTR2))
#define UPDATECONFSTR (abs (stride >= 64) ? (UPDATECONFSTR1 || UPDATECONFSTR1) : (UPDATECONFSTR1))
        return (UPDATECONFSTR &
	  ((abs (stride) > 1) || (!inst->isLoad()) //(U.INSTTYPE != loadInstClass)
	   || ((stride == -1) & ((random () & 1) == 0))
	   || ((stride == 1) & ((random () & 3) == 0))));
//All strides are not equal: the smaller the stride the smaller the benefit (not huge :-))

}

//Allocate or not if instruction absent from the predictor
bool
EvesLVP::StrideAllocateOrNot (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency)
{
return true;
#ifndef LIMITSTUDY
  bool X = false;
#define LOGPBINVSTR 4
 //  switch (U.INSTTYPE)
 //    {
 //    case aluInstClass:
 //    case storeInstClass:
 //      X = ((random () & ((1 << (LOGPBINVSTR + 2)) - 1)) == 0);
 //      break;
 //    case fpInstClass:
 //      X = ((random () & ((1 << (LOGPBINVSTR)) - 1)) == 0);
 //      break;
 //    case slowAluInstClass:
 //      X = ((random () & ((1 << (LOGPBINVSTR)) - 1)) == 0);
 //      break;
 //    case loadInstClass:
 //      X =
	// ((random () &
	//   ((1 << (NOTLLCMISS + NOTL2MISS + NOTL1MISS + MFASTINST)) - 1)) ==
	//  0);
 //      break;
 //    };
  // Turning this into if/else
  if (inst->isStore() || inst->isInteger()) {
  	X = ((random () & ((1 << (LOGPBINVSTR + 2)) - 1)) == 0);
  } else if (inst->isFloating()) {
  	X = ((random () & ((1 << (LOGPBINVSTR)) - 1)) == 0);
  } else if (isSlowALU(inst)) {
  	X = ((random () & ((1 << (LOGPBINVSTR)) - 1)) == 0);
  } else if (inst->isLoad()) {
  	X = ((random () & ((1 << (NOTLLCMISS + NOTL2MISS + NOTL1MISS + MFASTINST)) - 1)) == 0);
  } else {
  // presumably nothing happens otherwise
  }

  return (X);

#else
  return (true);
#endif
}

void
EvesLVP::UpdateStridePred (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency)
{


  int B[NBWAYSTR];
  int TAG[NBWAYSTR];
  for (int i = 0; i < NBWAYSTR; i++)
    {
      B[i] = U.B[i];
      TAG[i] = U.TAGSTR[i];
    }
  int STHIT = -1;

  for (int i = 0; i < NBWAYSTR; i++)
    {
      if (STR[B[i]].tag == TAG[i])
	{
	  STHIT = B[i];
	  break;

	}

    }





  if (STHIT >= 0)
    {
      uint64_t LastValue = STR[STHIT].LastValue;
      uint64_t Value =
	(uint64_t) ((int64_t) LastValue + (int64_t) STR[STHIT].Stride);
      int64_t INTER =
	abs (2 * ((int64_t) actual_value - (int64_t) LastValue) - 1);

      uint64_t stridetoalloc =
	(INTER <
	 (1 << LOGSTRIDE)) ? (uint64_t) ((int64_t) actual_value -
					 (int64_t) LastValue) : 0;

      STR[STHIT].LastValue = actual_value;

//special case when the stride is not determined
      if (STR[STHIT].NotFirstOcc > 0)
	{
	  if (Value == actual_value)
	    {

	      if (STR[STHIT].conf < MAXCONFIDSTR)
		{
		  if (strideupdateconf
		      (U, inst, actual_value, actual_latency, (int) stridetoalloc))
		    STR[STHIT].conf++;
		}

	      if (STR[STHIT].u < 3)
		if (strideupdateconf
		    (U, inst, actual_value, actual_latency, (int) stridetoalloc))
		  STR[STHIT].u++;
	      if (STR[STHIT].conf >= MAXCONFIDSTR / 4)
		STR[STHIT].u = 3;
	    }
	  else
	    {
//misprediction

	      {
		if (STR[STHIT].conf > (1 << (WIDTHCONFIDSTR - 3)))

		  {
		    STR[STHIT].conf -= (1 << (WIDTHCONFIDSTR - 3));
		  }
		else
		  {
		    STR[STHIT].conf = 0;
		    STR[STHIT].u = 0;
		  }


	      }

// this allows to  restart a new sequence with a different   stride              
	      STR[STHIT].NotFirstOcc = 0;
	    }
	}
      else
	{
//First occurence              
//        if (STR[STHIT].NotFirstOcc == 0)
	  if (stridetoalloc != 0)
//             if ((stridetoalloc != 0) && (stridetoalloc!=1)  && (((int64_t) stridetoalloc) != -1))
	    // we do not waste the stride predictor storage for stride zero
	    {
	      STR[STHIT].Stride = stridetoalloc;
	    }
	  else
	    {
	      // do not pollute the stride predictor with constant data or with invalid strides
	      STR[STHIT].Stride = 0xffff;
	      STR[STHIT].conf = 0;
	      STR[STHIT].u = 0;
	    }

	  STR[STHIT].NotFirstOcc++;
	}
    }
  else				// the address was not present and is not predicted by VTAGE
  if (!U.prediction_result)
    {
      if (StrideAllocateOrNot (U, inst, actual_value, actual_latency))
	{			// try to allocate
	  int X = random () % NBWAYSTR;
	  bool done = false;
	  // the target entry is not a stride candidate
	  for (int i = 0; i < NBWAYSTR; i++)
	    {
	      // if (i == 0) cout << "Find something unconfident" << endl;
	      STHIT = B[X];
	      if (STR[STHIT].conf == 0)
		{
		  STR[STHIT].conf = 1;	//just to allow not to ejected before testing if possible stride candidate
		  STR[STHIT].u = 0;
		  STR[STHIT].tag = TAG[X];
		  STR[STHIT].Stride = 0;
		  STR[STHIT].NotFirstOcc = 0;
		  STR[STHIT].LastValue = actual_value;
		  done = true;
		  break;

		}
	      X = (X + 1) % NBWAYSTR;
	    }
	  // the target entry has not been useful recently
	  if (!done)
	    for (int i = 0; i < NBWAYSTR; i++)
	      {
		// if (i == 0) cout << "Find something unuseful" << endl;
		STHIT = B[X];
		if (STR[STHIT].u == 0)
		  {
		    STR[STHIT].conf = 1;
		    STR[STHIT].u = 0;
		    STR[STHIT].tag = TAG[X];
		    STR[STHIT].Stride = 0;
		    STR[STHIT].NotFirstOcc = 0;
		    STR[STHIT].LastValue = actual_value;
		    done = true;
		    break;

		  }
		X = (X + 1) % NBWAYSTR;
	      }
//if unable to allocate: age some target entry
	  if (!done)
	    {  // cout << "STILL NOT DONE?!" << endl;
	      if ((random () &
		   ((1 <<
		     (2 + 2 * (STR[STHIT].conf > (MAXCONFIDSTR) / 8) +
		      2 * (STR[STHIT].conf >= MAXCONFIDSTR / 4))) - 1)) == 0)
		STR[STHIT].u--;
	    }

	}
    }
//////////////////////////////
}  // TODO: check for necessary refactoring



/////////Update of  VTAGE
// function determining whether to  update or not confidence on a correct prediction

bool
EvesLVP::vtageupdateconf (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency)
{
#define LOWVAL ((abs (2*((int64_t) actual_value)+1)<(1<<16))+ (actual_value==0))


#ifdef K8
#define updateconf ((random () & (((1 << (LOWVAL+NOTLLCMISS+2*FASTINST+NOTL2MISS+NOTL1MISS + ((!inst->isLoad()) ||NOTL1MISS)      + (U.HitBank > 1) ))- 1)))==0)
#else
#define updateconf ((random () & (((1 << (LOWVAL+NOTLLCMISS+2*FASTINST+NOTL2MISS+NOTL1MISS + ((!inst->isLoad()) ||NOTL1MISS)       ))- 1)))==0)
#endif

// jer5ae note: frankly again I have no idea why these macros are so repetitive
// might be worth emailing seznec about it...
#ifdef LIMITSTUDY
#define UPDATECONF2 ((U.HitBank<=1) ? (updateconf || updateconf ) || (updateconf || updateconf ) : (updateconf || updateconf ))
#define UPDATECONF (UPDATECONF2 || UPDATECONF2)
#else
#ifdef K32
#define UPDATECONF (((U.HitBank<=1) ? (updateconf || updateconf) : updateconf))
#else
  // K8
#define UPDATECONF updateconf
#endif
#endif
  // switch (U.INSTTYPE)
  //   {
  //   case aluInstClass:

  //   case fpInstClass:
  //   case slowAluInstClass:
  //   case undefInstClass:
  //   case loadInstClass:
  //   case storeInstClass:
  //     return (UPDATECONF);
  //     break;
  //   case uncondIndirectBranchInstClass:
  //     return (true);
  //     break;
  //   default:
  //     return (false);
  //   };
  
  if (inst->isInteger() || inst->isFloating() || inst->isLoad() || inst->isStore()
    	|| isSlowALU(inst)) {  // TODO: make sure it's ok to just ignore undefInstClass
  	return (UPDATECONF);
  } else if (!(inst->isCondCtrl() && inst->isIndirectCtrl())) {  // TODO: totally verify this
  	return true;
  }
  return false;
}

// Update of the U counter or not 
bool
EvesLVP::VtageUpdateU (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency)
{

#define UPDATEU ((!U.prediction_result) && ((random () & ((1<<( LOWVAL + 2*NOTL1MISS  + (!inst->isLoad()) + FASTINST + 2*(inst->isInteger())*(inst->numSrcRegs()<2)))-1)) == 0))
  // switch (U.INSTTYPE)
  //   {
  //   case aluInstClass:
  //   case fpInstClass:
  //   case slowAluInstClass:
  //   case undefInstClass:
  //   case loadInstClass:
  //   case storeInstClass:
  //     return (UPDATEU);
  //     break;
  //   case uncondIndirectBranchInstClass:
  //     return (true);
  //     break;
  //   default:
  //     return (false);
  //   };
  if (inst->isInteger() || inst->isFloating() || inst->isLoad() || inst->isStore()
    	|| isSlowALU(inst)) {  // TODO: make sure it's ok to just ignore undefInstClass
  	return (UPDATEU);
  } else if (!(inst->isCondCtrl() && inst->isIndirectCtrl())) {  // todo: verify this works (isIndirectCtrl)
  	return true;
  } else { return false; }

}

bool
EvesLVP::VtageAllocateOrNot (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency, bool MedConf)
{
  return true;
  bool X = false;

// TODO: this might need fine-tuning. Check with mypredictor.cc.
bool c = 1;
#ifndef LIMITSTUDY
if (inst->isInteger() || inst->isStore()) {
  c = ((random () & 15) == 0);
}
#endif

if (inst->isInteger() || inst->isFloating() || inst->isLoad() || inst->isStore()) {
  #ifndef LIMITSTUDY
  X = (((random () &
  #ifdef K8
           ((4 <<
  #else
           //K32
           ((2 <<
  #endif
       (
  #ifdef K32
         ((!inst->isLoad()) || (NOTL1MISS)) *
  #endif
         LOWVAL + NOTLLCMISS + NOTL2MISS +
  #ifdef K8
         2 *
  #endif
         NOTL1MISS + 2 * FASTINST)) - 1)) == 0) ||
  #ifdef K8
         (((random () & 3) == 0) && MedConf));
  #else
         (MedConf));
  #endif
  #else
    X = true;
  #endif
}

else if (inst->isCondCtrl() && inst->isIndirectCtrl()) X = true;
else {X = false;}
return X && c;
}  // TODO: this is downright arcane, even after rephrasing. Clean up.


void
EvesLVP::UpdateVtagePred (LVPredUnit::lvpReturnValues& U, StaticInstPtr inst, uint64_t actual_value, int actual_latency)
{

  bool MedConf = false;
  uint64_t HashData = ((actual_value ^ (actual_value >> 7) ^
			(actual_value >> 13) ^ (actual_value >> 21) ^
			(actual_value >> 29) ^ (actual_value >> 34) ^
			(actual_value >> 43) ^ (actual_value >> 52) ^
			(actual_value >> 57)) & (BANKDATA - 1)) +
    3 * BANKDATA;
  bool ShouldWeAllocate = true;
  if (U.HitBank != -1)
    {
      // there was  an  hitting entry in VTAGE
      // cout << "Accessing U.HitBank " << U.HitBank << " in U.GI" << endl;
      uint64_t index = U.GI[U.HitBank];
      // the entry may have dissappeared in the interval between the prediction and  the commit


      if (Vtage[index].tag == U.GTAG[U.HitBank])
	{
	  //  update the prediction
	  uint64_t indindex = Vtage[index].hashpt;
	  ShouldWeAllocate =
	    ((indindex >= 3 * BANKDATA) & (indindex != HashData))
	    || ((indindex < 3 * BANKDATA) &
		(LDATA[indindex].data != actual_value));
	  if (!ShouldWeAllocate)
	    {
	      // the predicted result is satisfactory: either a good hash without data, or a pointer on the correct data
	      ShouldWeAllocate = false;

	      if (Vtage[index].conf < MAXCONFID)
		if (vtageupdateconf (U, inst, actual_value, actual_latency))
		  Vtage[index].conf++;

	      if (Vtage[index].u < MAXU)
		if ((VtageUpdateU (U, inst, actual_value, actual_latency))
		    || (Vtage[index].conf == MAXCONFID))

		  Vtage[index].u++;
	      if (indindex < 3 * BANKDATA)
		if (LDATA[indindex].u < 3)
		  if (Vtage[index].conf == MAXCONFID)
		    LDATA[indindex].u++;


	      if (indindex >= 3 * BANKDATA)
		{

		  //try to allocate an effective data entry when the confidence has reached a reasonable level
		  if (Vtage[index].conf >= MAXCONFID - 1)
		    {
		      int X[3];
		      for (int i = 0; i < 3; i++)
			X[i] =
			  (((actual_value) ^
			    (actual_value >> (LOGLDATA + (i + 1))) ^
			    (actual_value >> (3 * (LOGLDATA + (i + 1)))) ^
			    (actual_value >> (4 * (LOGLDATA + (i + 1)))) ^
			    (actual_value >> (5 * (LOGLDATA + (i + 1)))) ^
			    (actual_value >> (6 * (LOGLDATA + (i + 1)))) ^
			    (actual_value >> (2 * (LOGLDATA + (i + 1))))) &
			   ((1 << LOGLDATA) - 1)) + i * (1 << LOGLDATA);
		      bool done = false;
		      for (int i = 0; i < 3; i++)
			{
			  if (LDATA[X[i]].data == actual_value)
			    {
			      //the data is already present
			      Vtage[index].hashpt = X[i];
			      done = true;
			      break;
			    }
			}
		      if (!done)
			if ((random () & 3) == 0)
			  {
			    //data absent: let us try try to steal an entry
			    int i = (((uint64_t) random ()) % 3);
			    done = false;
			    for (int j = 0; j < 3; j++)
			      {
				if ((LDATA[X[i]].u == 0))
				  {
				    LDATA[X[i]].data = actual_value;
				    LDATA[X[i]].u = 1;
				    Vtage[index].hashpt = X[i];
				    done = true;
				    break;
				  }
				i++;
				i = i % 3;

			      }
			    if (inst->isLoad())
			      if (!done)
				{
				  if ((LDATA[X[i]].u == 0))
				    {
				      LDATA[X[i]].data = actual_value;
				      LDATA[X[i]].u = 1;
				      Vtage[index].hashpt = X[i];
				    }
				  else
#ifdef K8
				  if ((random () & 31) == 0)
#else
				  if ((random () & 3) == 0)
#endif
				    LDATA[X[i]].u--;
				}
			  }
		    }
		}



	    }

	  else
	    {
// misprediction: reset

	      Vtage[index].hashpt = HashData;
	      if ((Vtage[index].conf > MAXCONFID / 2)
		  || ((Vtage[index].conf == MAXCONFID / 2) &
		      (Vtage[index].u == 3))
		  || ((Vtage[index].conf > 0) &
		      (Vtage[index].conf < MAXCONFID / 2)))
		MedConf = true;

	      if (Vtage[index].conf == MAXCONFID)
		{

		  Vtage[index].u = (Vtage[index].conf == MAXCONFID);
		  Vtage[index].conf -= (MAXCONFID + 1) / 4;
		}
	      else
		{
		  Vtage[index].conf = 0;
		  Vtage[index].u = 0;
		}

	    }
	}
    }

  if (!U.prediction_result)
    //Don't waste your time allocating if it is predicted by the other component
    if (ShouldWeAllocate)
      {
// avoid allocating too often
	if (VtageAllocateOrNot (U, inst, actual_value, actual_latency, MedConf))
	  {
	    int ALL = 0;
	    int NA = 0;
	    int DEP = (U.HitBank + 1) + ((random () & 7) == 0);
	    if (U.HitBank == 0)
	      DEP++;

	    if (U.HitBank == -1)
	      {
		if (random () & 7)
		  DEP = random () & 1;
		else
		  DEP = 2 + ((random () & 7) == 0);

	      }

	    if (DEP > 1)
	      {


		for (int i = DEP; i <= NHIST; i++)
		  {
		    int index = U.GI[i];
		    if ((Vtage[index].u == 0)
			&& ((Vtage[index].conf == MAXCONFID / 2)
			    || (Vtage[index].conf <=
				(random () & MAXCONFID))))
//slightly favors the entries with real confidence
		      {
			Vtage[index].hashpt = HashData;
			Vtage[index].conf = MAXCONFID / 2;	//set to 3  for faster warming to  high confidence 
			Vtage[index].tag = U.GTAG[i];
			ALL++;

			break;

		      }
		    else
		      {
			NA++;
		      }

		  }
	      }
	    else
	      {

		for (int j = 0; j <= 1; j++)
		  {
		    int i = (j + DEP) & 1;

		    int index = U.GI[i];
		    if ((Vtage[index].u == 0)
			&& ((Vtage[index].conf == MAXCONFID / 2)
			    || (Vtage[index].conf <=
				(random () & MAXCONFID))))
		      {
			Vtage[index].hashpt = HashData;
			Vtage[index].conf = MAXCONFID / 2;
			if (true)//inst->numSrcRegs() == 0)
			  if (inst->isInteger())
			    Vtage[index].conf = MAXCONFID;
			Vtage[index].tag = U.GTAG[i];
			ALL++;
			break;
		      }
		    else
		      {
			NA++;
		      }
		  }
	      }

#ifdef K8
	    TICK += NA - (3 * ALL);
#else
	    TICK += (NA - (5 * ALL));
#endif
	    if (TICK < 0)
	      TICK = 0;
	    if (TICK >= MAXTICK)
	      {

		for (int i = 0; i < PREDSIZE; i++)
		  if (Vtage[i].u > 0)
		    Vtage[i].u--;
		TICK = 0;
	      }
	  }

      }


}


bool
// updatePredictor (StaticInstPtr inst, //(uint64_t seq_no,
// 		 uint64_t actual_addr, uint64_t actual_value, uint64_t actual_latency)
EvesLVP::processPacketRecieved(TheISA::PCState actual_addr, StaticInstPtr inst,
	uint64_t actual_value, ThreadID tid, uint64_t predictedValue, int8_t confidence,
	unsigned actual_latency, unsigned currentCycle)
{
  // ForUpdate *U;
  // U = &Update[seq_no & (MAXINFLIGHT - 1)];
	LVPredUnit::lvpReturnValues U = LVPredUnit::lvpReturnValues(inst->predictedValue, inst->confidence);
	U.predVtage = inst->predVtage;
	U.predStride = inst->predStride;
	//if (U.predStride) {std::cout << "LATE CONFIDENT!\n";}
        //U.prediction_result = inst->prediction_result;
	U.prediction_result = (U.predictedValue == actual_value);
	
        if (confidence > 0) {  // if it was a "confident prediction"...
	    if (U.predictedValue == actual_value) {
		correctUsed++;
		cyclesSaved += actual_latency;
	    } else {
		incorrectUsed++;
	    }
        } else {
	    if (U.predictedValue == actual_value) {
		correctNotUsed++;
	    } else {
		incorrectNotUsed++;
	    }
	}

	for (int i=0; i<9; i++) {
		U.GTAG[i] = inst->GTAG[i];
		U.GI[i] = inst->GI[i];
	}
	for (int i=0; i<3; i++) {
		U.TAGSTR[i] = inst->TAGSTR[i];
		U.B[i] = inst->B[i];
	}
	U.STHIT = inst->STHIT;
	// cout << "Setting U.HitBank to inst value " << inst->HitBank << endl;
	U.HitBank = inst->HitBank;

  if (true) 
    {
#ifdef LIMITSTUDY
      //just to force allocations and update on both predictors
      U.prediction_result = 0;
#endif
#ifdef PREDVTAGE
      UpdateVtagePred (U, inst, actual_value, (int) actual_latency);
#endif
#ifdef PREDSTRIDE
      UpdateStridePred (U, inst, actual_value, (int) actual_latency);
#endif
      //U.todo = 0;
   }
  seq_commit = stored_seq_no;  // this might cause big problems
  // return true iff no misp occurred
  return (U.prediction_result || !(U.predVtage || U.predStride));
}


// TODO. What happens if we just don't use this code?
/*

void  // It looks like this goes with wherever lvpred update occurs, but notably, OUTSIDE the lv predictor.
speculativeUpdate (uint64_t seq_no,	// dynamic micro-instruction # (starts at 0 and increments indefinitely)
		   bool eligible,	// true: instruction is eligible for value prediction. false: not eligible.
		   uint8_t prediction_result,	// 0: incorrect, 1: correct, 2: unknown (not revealed)
		   // Note: can assemble local and global branch history using pc, next_pc, and insn.
		   uint64_t
		   pc, uint64_t next_pc, InstClass insn, uint8_t piece,
		   // Note: up to 3 logical source register specifiers, up to 1 logical destination register specifier.
		   // 0xdeadbeef means that logical register does not exist.
		   // May use this information to reconstruct architectural register file state (using log. reg. and value at updatePredictor()).
		   uint64_t src1, uint64_t src2, uint64_t src3, uint64_t dst)
{

  // the framework does not really allow  to filter the predictions, so we predict every instruction
  ForUpdate *U;
  U = &Update[seq_no & (MAXINFLIGHT - 1)];

  LastMispVT++;
  
  if (eligible)
    {

      U.NbOperand =
	(src1 != 0xdeadbeef) + (src2 != 0xdeadbeef) + (src3 != 0xdeadbeef);
      U.todo = 1;
      U.INSTTYPE = insn;
      U.prediction_result = (prediction_result == 1);
      if (SafeStride < (1 << 15) - 1)
	SafeStride++;
      if (prediction_result != 2)
	{
	  if (prediction_result)
	    {
	      if (U.predstride)
		if (SafeStride < (1 << 15) - 1)
		  SafeStride += 4 * (1 + (insn == loadInstClass));
	    }
	  else
	    {
	      if (U.predvtage)
		LastMispVT = 0;
	      if (U.predstride)
		SafeStride -= 1024;
	    }  // TODO: I *think* we probably just need to update SafeStride and LastMispVT.
	    // Both of these can be viewed as optimizations, though; let's do them later.
	}
    }

  bool isCondBr = insn == condBranchInstClass;
  bool isUnCondBr = insn == uncondIndirectBranchInstClass
    || insn == uncondDirectBranchInstClass;
//path history 
  // just to have a longer history without (software) management
  if ((isCondBr) || (isUnCondBr))
    if (pc != next_pc - 4)
      {
	for (int i = 7; i > 0; i--)
	  gpath[i] = (gpath[i] << 1) ^ ((gpath[i - 1] >> 63) & 1);
	gpath[0] = (gpath[0] << 1) ^ (pc >> 2);
	gtargeth = (gtargeth << 1) ^ (next_pc >> 2);
	// TODO: perhaps implement this as part of the update in fetch_impl?
      }
}

void
beginPredictor (int argc_other, char **argv_other)
{
}

void
endPredictor ()
{
// looks like this is only used to print out a summary
#ifndef LIMITSTUDY
    int SIZE = 0;
    SIZE = NBWAYSTR * (1 << LOGSTR) * (67 + LOGSTRIDE + TAGWIDTHSTR + WIDTHCONFIDSTR) + 16;	//the SafeStride counter
    printf ("STORAGE SIZE: STRIDE (%d bits)", SIZE);

    int INTER = (((64 - LOGLDATA) + 2) * 3 << LOGLDATA);	//the 64 bits data words - LOGLDATA implicits bits  + 2 u bits
    printf (" |Value array:  (%d bits)", INTER);
    SIZE += INTER;
    INTER = BANKSIZE * NBBANK * (TAGWIDTH + (LOGLDATA + 2) + WIDTHCONFID + UWIDTH)	//the VTAGE entries
      + 8			// the LastMispVT counter
      + 10;			// the TICK counter

    printf (" |VTAGE:  (%d bits)", INTER);
    SIZE += INTER;
    printf (" ||| TOTAL SIZE: %d bits\n", SIZE);
#endif
}

*/

void
EvesLVP::updateGtables(Addr pc, Addr next_pc, bool branches)
{
return;
// adding in the LastMispVT should be easy if this works
    if (branches && (pc != next_pc - 4)) {
    	for (int i = 7; i > 0; i--)
	  gpath[i] = (gpath[i] << 1) ^ ((gpath[i - 1] >> 63) & 1);
	gpath[0] = (gpath[0] << 1) ^ (pc >> 2);
	gtargeth = (gtargeth << 1) ^ (next_pc >> 2);
    }
}
