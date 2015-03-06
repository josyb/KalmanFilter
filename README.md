# KalmanFilter
A simple low-resource usage Kalman Filter using shared resources   and 
Written with [MyHDL](http://myhdl.org): **the future of HDL**  

I was faced to filter a noisy temperature measurement, but it was for a _legacy_ Altera EP1C6 device, without Hardware Multipliers.  
There was no real speed requirement as the filter was to feed a PID running alt a slow rate of 10 to 50Hz. So I opted to use a bit-serial multiplier and re-use this for the 3 calculations. As there where also additions to be done, I wanted to re-use the adder in the multiplier too.  
After a while it turned out that we can get away with one large adder to do the multiplications **and** to do the additions without actually doing the additions :)

### Kalman Filter Theory (abbr.)
There are plentyful resources on the web, so I'm not going to cover much more than the bare necessities to explain what I've done.  
The basic formula is:  
```text
y(n+1)  = a.y(n) + b.x(n)
```
For a _true_ Kalman Filter the coefficients _a_ and _b_ are variable and adapted in time.  
We don't have the resources, as we are using an EP1C6 device without multipliers, and we wanted to keep the resource usage and develoment time low we implemented the filter with a fixed _gain_. It then efectively becomes a low pass filter.
We use fixed values for _a_ and _b_, in this case 0.99121 and 0.000879. As we work in binary, they become 1015 / 1024 and 9 / 1024  

t.b.c. ...
