These files are the c++ implementation of the orientation estimation test.


The test is performed in different orientation estimation algorithms:
- OrientationEstimator > MEKF > MEKFcO
- OrientationEstimator > MEKF > MEKFcRP
- OrientationEstimator > MEKF > MEKFcMRP
- OrientationEstimator > MEKF > MEKFcRV
- OrientationEstimator > MUKF > MUKFcO
- OrientationEstimator > MUKF > MUKFcRP
- OrientationEstimator > MUKF > MUKFcMRP
- OrientationEstimator > MUKF > MUKFcRV
- OrientationEstimator > MadgwickAHRS

Some configuration parameters are defined in
- config.h


The test is implemented in:
- OE_Tester
