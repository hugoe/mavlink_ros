/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */

#ifndef MAVLINK_MATH_H_
#define MAVLINK_MATH_H_

// from asctec_hl_interface
inline void angle2quaternion(const double &roll, const double &pitch, const double &yaw, double *w, double *x,
                             double *y, double *z)
{
  double sR2, cR2, sP2, cP2, sY2, cY2;
  sincos(roll * 0.5, &sR2, &cR2);
  sincos(pitch * 0.5, &sP2, &cP2);
  sincos(yaw * 0.5, &sY2, &cY2);

  // TODO: change rotation order
  // this follows AscTec's pre- 2012 firmware rotation order: Rz*Rx*Ry
//  *w = cP2 * cR2 * cY2 - sP2 * sR2 * sY2;
//  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
//  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
//  *z = cP2 * cR2 * sY2 + cY2 * sP2 * sR2;

  // Rz*Ry*Rx for 2012 firmware on the:
  *w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
  *z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
}

#endif /* MAVLINK_MATH_H_ */
