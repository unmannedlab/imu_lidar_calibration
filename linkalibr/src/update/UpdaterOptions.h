//
// Created by usl on 11/28/20.
//

#ifndef LIN_ESTIMATOR_UPDATEROPTIONS_H
#define LIN_ESTIMATOR_UPDATEROPTIONS_H

namespace lin_estimator {
  /// Struct which stores general updater options
  struct UpdaterOptions {
      /// What chi2_multiploer should we apply
      int chi2_multiplier = 5;

      /// Noise levels
      double noise_translation  = 0.1;
      double noise_rotation = 0.1;
      /// Do chi2 check?
      bool do_chi2_check = true;

      /// Print function of what parameters we have loaded
      void print() {
          printf("\t- chi2 multiplier: %d\n", chi2_multiplier);
          printf("\t- Noise translation: %f\n", noise_translation);
          printf("\t- Noise rotation: %f\n", noise_rotation);
          printf("\t- Do chi2 check during update: %d\n", do_chi2_check);
      }
  };
};
#endif //LIN_ESTIMATOR_UPDATEROPTIONS_H
