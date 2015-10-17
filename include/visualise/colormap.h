/*
 * colormap.h
 *
 *  Created on: Aug 7, 2012
 *      Author: guillaume
 */

#ifndef COLORMAP_H_
#define COLORMAP_H_

#include <vector>

typedef void (*colorMapFunc)(unsigned char*,float,float,float);

class ColorMap{
 public:
  static void colorMap(unsigned char *rgb,float value,float min,float max);
  static void hotColorMap(unsigned char *rgb,float value,float min,float max);
  static void coldColorMap(unsigned char *rgb,float value,float min,float max);
  static void jetColorMap(unsigned char *rgb,float value,float min,float max);
  static void cyclicColorMap(unsigned char *rgb,float value,float min,float max);
  static void randColorMap(unsigned char *rgb,float value,float min,float max);
  static void grayColorMap(unsigned char *rgb,float value,float min,float max);
  static void blueColorMap(unsigned char *rgb,float value,float min,float max);
  static colorMapFunc selectColorMap(int cmp);

};

#endif /* COLORMAP_H_ */
