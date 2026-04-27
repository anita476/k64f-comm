# K64f Communication 

Communication protocol drivers/practice using k64f

### UART 

Obs! For UART must use flags for each implementation:
```make
-DUART_IMPL_POLLING # polling blocking implementation
-DUART_IMPL_NONBLOCKING # interrupt based implementation 
-DUART_IMPL_FIFO # interrupt based implementation with hardware fifo usage
```

> In MCUXpresso go to Projects > Properties > C/C++ Build > Settings > MCU C Compiler > Preprocessor and add the desired flags under `defined symbols`