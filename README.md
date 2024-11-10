# Kalman Filter Implementation in ARM Assembly, C, and CMSIS-DSP for STM32L4+ Series

## Introduction

This project involves the implementation of a one-dimensional Kalman filter using ARM Cortex M4 assembly language and compares its performance to C and CMSIS-DSP approaches. By analyzing state tracking, standard deviation, and data correlation, we evaluate the overall performance of each approach on the STM32 microcontroller.

## Table of Contents

- [Introduction](#introduction)
- [Project Overview](#project-overview)
- [Implementation Details](#implementation-details)
  - [ARM Assembly Implementation](#arm-assembly-implementation)
  - [C Implementation](#c-implementation)
  - [CMSIS-DSP Implementation](#cmsis-dsp-implementation)
- [Statistical Analysis](#statistical-analysis)
- [Results](#results)
- [Conclusion](#conclusion)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Project Overview

The goal of this project is to:

- Implement a Kalman filter in ARM assembly language using floating-point operations.
- Integrate the assembly subroutine with C code and CMSIS-DSP.
- Compare the performance of the three implementations in terms of execution time and accuracy.
- Analyze the statistical properties of the filter outputs.

## Implementation Details

### ARM Assembly Implementation

In the ARM assembly implementation, we:

- Created an `update()` function that follows ARM's calling conventions.
- Used floating-point S-registers for the Kalman filter states.
- Implemented exception handling by checking the Floating-Point Status and Control Register (FPSCR).
- Ensured the subroutine can be integrated with C code.

#### Key Features

- **Floating-Point Operations**: Utilizes S-registers for precise calculations.
- **Exception Handling**: Checks for overflow, underflow, division by zero, and invalid operations.
- **Performance**: Optimized for ARM Cortex M4 architecture.

### C Implementation

For the C implementation, we:

- Developed a C version of the `update()` function using a `struct` to store Kalman states.
- Used standard arithmetic operations for state updates.
- Accessed the FPSCR for exception handling through a `get_FPSCR()` method.
- Integrated the function seamlessly with the main C codebase.

#### Key Features

- **Simplicity**: Uses standard C syntax and operations.
- **Portability**: Easier to port across different platforms supporting C.
- **Exception Handling**: Similar approach to assembly implementation.

### CMSIS-DSP Implementation

In the CMSIS-DSP implementation, we:

- Leveraged the CMSIS-DSP library functions optimized for ARM processors.
- Adapted vector-optimized functions for scalar operations by setting the length parameter to 1.
- Implemented the Kalman filter using built-in functions for arithmetic operations.

#### Key Features

- **Library Functions**: Utilizes optimized CMSIS-DSP functions.
- **Ease of Use**: Simplifies code by using pre-built functions.
- **Optimization**: Benefits from ARM's DSP optimizations.

## Statistical Analysis

We performed a statistical analysis to compare the implementations based on:

- **Execution Time**: Measured the average run time over 1000 iterations.
- **Accuracy**: Calculated the mean and standard deviation of the difference between input and output measurements.
- **Correlation**: Assessed how closely the output follows the input signal.

### Timing Results

| **Algorithm**            | **Time per 1000 Iterations (s)** | **Average Time per Iteration (ms)** |
|--------------------------|----------------------------------|-------------------------------------|
| C Implementation         | 0.103375533                      | 0.103                               |
| Assembly Implementation  | 0.147164825                      | 0.147                               |
| CMSIS-DSP Implementation | 0.342527317                      | 0.343                               |

### Accuracy Metrics

| **Metric**           | **C Implementation** | **CMSIS-DSP Implementation** |
|----------------------|----------------------|------------------------------|
| Average Difference   | 0.024999015          | 0.024999015                  |
| Standard Deviation   | 0.232199296          | 0.233357385                  |

### Correlation and Convolution

We computed the correlation and convolution between the input and output signals to assess the filter's performance:

- **Correlation**: High correlation indicates the output closely follows the input.
- **Convolution**: Helps in understanding how the input signal is modified by the filter.

Graphs illustrating these analyses can be found in the `graphs` directory.

## Results

All three implementations produced similar outputs, effectively demonstrating the Kalman filter's properties such as convergence towards the input signal. The key observations are:

- **Execution Time**: The C implementation was the fastest, followed by assembly, with CMSIS-DSP being the slowest due to library overhead.
- **Accuracy**: All implementations had nearly identical outputs, with minor differences in standard deviation due to floating-point precision.
- **Exception Handling**: Both assembly and C implementations effectively handled exceptions via FPSCR.

## Conclusion

Our comparative analysis reveals:

- The **C implementation** offers the best performance in terms of execution time while maintaining accuracy.
- The **Assembly implementation** is slightly slower but could be optimized further for speed improvements.
- The **CMSIS-DSP implementation** provides correct results but has a longer runtime due to the overhead associated with the CMSIS library functions.

The choice of implementation depends on specific project requirements:

- **Performance-Critical Applications**: Assembly or optimized C implementations are preferred.
- **Ease of Development**: C implementation offers simplicity and portability.
- **Leveraging Library Functions**: CMSIS-DSP is suitable when built-in functions and DSP optimizations are desired.

## Authors

- **Samer Abdulkarim**
  - Department of Electrical, Computer, Software Engineering
  - McGill University, Canada
  - Student ID: 260964596
  - Email: [samer.abdulkarim@mail.mcgill.ca](mailto:samer.abdulkarim@mail.mcgill.ca)
- **Raphael Verger**
  - Department of Electrical, Computer, Software Engineering
  - McGill University, Canada
  - Student ID: 261030528
  - Email: [raphael.verger@mail.mcgill.ca](mailto:raphael.verger@mail.mcgill.ca)

## Acknowledgments

This project was completed as part of **ECSE 444 - Laboratory No. 1** at McGill University.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

*Note: For detailed graphs, code samples, and further explanations, please refer to the respective directories and files within this repository.*
