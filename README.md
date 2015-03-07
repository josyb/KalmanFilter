# KalmanFilter
A simple low-resource usage Kalman Filter using shared resources   
Written with [MyHDL](http://myhdl.org): **the future of HDL**  

I was faced to filter a noisy temperature measurement, but it was for a _legacy_ Altera EP1C6 device, without Hardware Multipliers.  
There was no real speed requirement as the filter was to feed a PID running alt a slow rate of 10 to 50Hz. So I opted to use a bit-serial multiplier and re-use this for the 3 calculations. As there where also additions to be done, I wanted to re-use the adder from the multiplier to do these as well.  
After a while it turned out that we can get away with one large adder to do the multiplications **and** to do the additions without actually doing the additions :)

### Kalman Filter Theory (abbr.)
There are plentyful resources on the web, so I'm not going to cover much more than the bare necessities to explain what I've done.  
The basic formula is:  
```text
y(n)  = a.y(n-1) + b.x(n)
```
For a _true_ Kalman Filter the coefficients _a_ and _b_ are variable and adapted in time.  
We don't have the resources, as we are using an EP1C6 device without multipliers. And we wanted to keep the resource usage and develoment time low, so we implemented the filter with a fixed _gain_. It then efectively becomes a low pass filter.
We use _fixed_ values for _a_ and _b_, in this case 0.99121 and 0.000879. The sum of _a_ and _b_ is 1. If smaller the filter doesn't reach the _average_ value. If it is more it will overshoot, or oscillate.

### HDL'rs do it integer
The above formula is normally calculated in _Floating Point Arithemetic_ which at best is problematic in any FPGA except the latest $1k+ devices ...  
Instead we use _'Scaled Integer Arithmetic'_. A scaling factor of about 1000 is usually a good starting point, so we chose to scale by 1024. The above coefficients then become 1015 and 9 respectively. 
The formula becomes then:
```text
y(n) = (a.y(n-1) + b.x(n)) / 1024
```
The division by 1024, the 'scaling', has the side efect of rounding (by truncation) towards _negative infinity_, so we will make it round to the nearest integer by adding half of the scaling factor before doing the scaling operation:
```text
y(n) = (a.y(n-1) + b.x(n) + 1024/2 ) / 1024
```

t.b.c. ...
