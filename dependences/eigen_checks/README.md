Eigen check macros
============

A repository for glog CHECK and gtest ASSERT and EXPECT macros for Eigen types.

Licensed under the 3-clause BSD license ("New BSD").

```
Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## GLOG
The Glog `CHECK()` macros are used to check errors and exit your program if they are found. These Eigen equivalents can be used to check conditions on matrices. See [the glog documentation](https://google-glog.googlecode.com/svn/trunk/doc/glog.html) for a full description. The general form of use is:

```c++
CHECK_EIGEN_MATRIX_EQUAL(MatrixA, MatrixB) << "Informative error message!";
```
All tests happen component-wise. They fail if the matrices are different sizes. If the matrices are the same size, the test is applied to each corresponding pair of components.

To get these macros, use:

```c++
#include<eigen-checks/glog.h>
```

#### `CHECK_EIGEN_MATRIX_EQUAL(MatrixA, MatrixB)`

Checks if two matrices are binary equal.

#### `CHECK_EIGEN_MATRIX_EQUAL_DOUBLE(MatrixA, MatrixB)`

Checks if two matrices are equal to floating-point precision

#### `CHECK_EIGEN_MATRIX_NEAR(MatrixA, MatrixB, Precision)`

Checks if two matrices are equal to a user-specified precision.

#### `CHECK_EIGEN_MATRIX_ZERO(MatrixA, Precision)`

Checks if a matrix is equal to zero to a user-specified precision.

## GTEST
The gtest macros are built to facilitate unit testing with matrix types. This library provides two pieces of functionality: an macro that defines the main function, or *entrypoint*, for a gtest invocation, and several macros for testing if matrices are similar.

### GTest 
The Gtest `EXPECT()` and `ASSERT()` macros are used to verify assumptions in your tests and print useful information in case these don't match the expected outcome. These Eigen equivalents can be used to check conditions on matrices. See [the gtest documentation](https://code.google.com/p/googletest/wiki/Primer#Assertions) for a full description. The general form of use is:

```c++
EXPECT_TRUE(EIGEN_MATRIX_EQUAL(MatrixA, MatrixB));
EXPECT_FALSE(EIGEN_MATRIX_EQUAL(MatrixA, MatrixB));
ASSERT_TRUE(EIGEN_MATRIX_EQUAL(MatrixA, MatrixB));
ASSERT_FALSE(EIGEN_MATRIX_EQUAL(MatrixA, MatrixB));
```

All tests happen component-wise. They fail if the matrices are different sizes. If the matrices are the same size, the test is applied to each corresponding pair of components.

To get these macros, use:

```c++
#include<eigen-checks/gtest.h>
```

#### `EXPECT_TRUE(EIGEN_MATRIX_EQUAL(MatrixA, MatrixB))`

Succeeds if two matrices are binary equal.

#### `EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(MatrixA, MatrixB))`

Succeeds if two matrices are equal to floating-point precision

#### `EXPECT_TRUE(EIGEN_MATRIX_NEAR(MatrixA, MatrixB, Precision))`

Succeeds if two matrices are equal to a user-specified precision.

#### `EXPECT_TRUE(EIGEN_MATRIX_ZERO(MatrixA, Precision))`

Succeeds if a matrix is equal to zero to a user-specified precision.


### Entrypoint

Instead of writing the following code in your unit-test runner file:

```c++
#include <gtest/gtest.h>
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  //...
  return RUN_ALL_TESTS();
}
```
You can just use:

```c++
#include <eigen-checks/entrypoint.h>

UNITTEST_ENTRYPOINT
```
