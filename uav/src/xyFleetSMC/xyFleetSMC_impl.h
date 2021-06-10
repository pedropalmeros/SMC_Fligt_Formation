// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file xyFleetSMC_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef xyFleetSMC_IMPL_H
#define xyFleetSMC_IMPL_H

#include <Object.h>

namespace flair {
namespace core {
class Matrix;
class io_data;
}
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
}
namespace filter {
class xyFleetSMC;
}
}

/*! \class xyFleetSMC_impl
* \brief Class defining a PID
*/

class xyFleetSMC_impl {
public:
  xyFleetSMC_impl(flair::filter::xyFleetSMC *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~xyFleetSMC_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void Data4Formation(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float i;
  bool first_update;
  void OwnSat(float &signal, double saturation_value);
  float SatVal(float value, double saturation_value);

  float FormationProtocol(int id, std::vector<float> &states);
  float FormationProtocol(int id, std::vector<float> &states, std::vector<float> &reldist);
  void Sat(float &signal, float satVal);
  float Sign(float value);
  float SignH(float value);

  int Form_index = 0;

private:
  flair::filter::xyFleetSMC *self;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;
  flair::gui::DoubleSpinBox *alpha_1, *alpha_2, *f1;
  flair::gui::DoubleSpinBox *sat_u;
  flair::gui::DoubleSpinBox *sat_ec, *_gain, *_gainFleet, *_gainL;

  flair::gui::DoubleSpinBox *FGain_01;
  flair::gui::DoubleSpinBox *sat_consensus;

  flair::gui::DoubleSpinBox *alpha, *beta, *rho;
  flair::gui::DoubleSpinBox *scaleCtrl;
  flair::gui::DoubleSpinBox *satCtrl;


};

#endif // xyFleetSMC_impl_H
