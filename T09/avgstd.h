/**
 * Library to calculate the average and standard deviation of a 
 * reading iteratively: without storing the whole data set.
 * 
 * Stores: 
 * double average
 * double variance
 * int numberOfReadings
 * double min
 * double max
 **/
 
 /* 
Copyright (c) <year> <copyright holders>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

 #ifndef AVGSTD_H
 #define AVGSTD_H

 #include "Arduino.h"

class AvgStd {
  public:
    float getMean();
    float getStd();
    float getMin();
    float getMax();
    float getVariance();
    unsigned int getN();
    void reset();
    void addReading(float);
    void checkAndAddReading(float);
    void setRejectionSigma(float);
    int getTrend();
    void setTrendInterval(int);
    void setSamplingInterval(int);
    AvgStd();
  private:
    float min, max, var, avg, r_sigma;
    unsigned int N;
  
  };

 #endif
