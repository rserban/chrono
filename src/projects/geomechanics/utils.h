#ifndef DEMOS_UTILS_H
#define DEMOS_UTILS_H

#include <iostream>

#include "chrono_multicore/physics/ChSystemMulticore.h"

// =============================================================================
// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.

static inline void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
  if ((x != n) && (x % (n / 100 + 1) != 0))
    return;

  float ratio = x / (float)n;
  unsigned int c = (unsigned int)(ratio * w);

  std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
  for (unsigned int i = 0; i < c; i++)
    std::cout << "=";
  for (unsigned int i = c; i < w; i++)
    std::cout << " ";
  std::cout << "]\r" << std::flush;
}

// =============================================================================
// Utility function to print to console a few important step statistics

static inline void TimingOutput(chrono::ChSystem* mSys, std::ofstream* ofile = NULL) {
  double TIME = mSys->GetChTime();
  double STEP = mSys->GetTimerStep();
  double BROD = mSys->GetTimerCollisionBroad();
  double NARR = mSys->GetTimerCollisionNarrow();
  double SOLVER = mSys->GetTimerLSsolve();
  double UPDT = mSys->GetTimerUpdate();
  double RESID = 0;
  int REQ_ITS = 0;
  int BODS = mSys->GetNumBodies();
  int CNTC = mSys->GetNumContacts();
  if (chrono::ChSystemMulticore* mc_sys = dynamic_cast<chrono::ChSystemMulticore*>(mSys)) {
      RESID = std::static_pointer_cast<chrono::ChIterativeSolverMulticore>(mSys->GetSolver())->GetResidual();
      REQ_ITS = std::static_pointer_cast<chrono::ChIterativeSolverMulticore>(mSys->GetSolver())->GetIterations();
      BODS = mc_sys->GetNumBodies();
      CNTC = mc_sys->GetNumContacts();
  }

  if (ofile) {
      char buf[200];
      sprintf(buf, "%8.5f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7d  %7d  %7d  %7.4f\n", TIME, STEP, BROD, NARR, SOLVER,
              UPDT, BODS, CNTC, REQ_ITS, RESID);
      *ofile << buf;
  }

  printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n", TIME, STEP, BROD, NARR, SOLVER,
         UPDT, BODS, CNTC, REQ_ITS, RESID);
}

#endif
