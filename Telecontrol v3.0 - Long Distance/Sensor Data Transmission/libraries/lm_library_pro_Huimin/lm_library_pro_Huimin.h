/*
 * lm_library_pro.h
 *
 *  Created on: Mar 18, 2025
 *      Author: Valentina Condorelli
 *      revised by Federico Masiero
 *
 *  Last Revision: April 9, 2025
 */
#pragma once
#ifndef LM_LIBRARY_PRO_HUIMIN_H
#define LM_LIBRARY_PRO_HUIMIN_H

/*DEPENDENCIES*/
#include <ArduinoEigen.h> // to change into #include <Eigen/Dense> if working with C++ on a PC workstation
#include "cminpack.h"     // library for least-square minimization
#include <time.h>

/*DEFINITIONS*/
// for serial communication                         
#define SERIAL_HEADER 0xAA                              // 170, for serial communication
#define SERIAL_TAIL   0xBB                              // 187, for serial communication

// for localization
#define NUM_SENSORS   2//8                                 // from sensor 0 to sensor NUM_SENSORS-1
#define NUM_MAGNETS   1
#define THETA         2 * PI / NUM_SENSORS              // used to build Sensor Position of a Ring
#define RING_RADIUS   68.1                              // rings radius where the sensors lie
#define M_COEFF       0.1016                            // dipole moment intensity (A*m^2)
#define EPS_IDENTITY (1E-7)                             // for Cylinder model
#define AXIS_TH      (1E-6)                             // for Cylinder model
#define POSE_PARAMS   3                                 // 6: for dipole, 7: for cylinder model
                                                        // 3 for Huimin Wrist Device

/*DECLARARATIONS*/

extern uint8_t sensorPorts[NUM_SENSORS];                // Connection with MUX and sensors
extern Eigen::MatrixXf sensorData;                      // Sensor field recordings matrix on sensor local axes (NUM_SENSORSx3)             
extern Eigen::MatrixXf sensorMagneticField;             // Sensor field recordings matrix on detector global axes (NUM_SENSORSx3)
extern Eigen::MatrixXf sensorField;
extern Eigen::MatrixXf sPos;                            // Sensor position matrix (NUM_SENSORSx3)
extern std::array<Eigen::MatrixXf, NUM_SENSORS+1> Rsens;// Rotation Matrix from sensor frame to Detector reference frame

// Ring parameters
extern Eigen::Matrix<float, 3, 1> d0;                   // Position vector of sensor 0
extern Eigen::MatrixXf wRs1, wRs2, wRs3;                // Rotation matrix from Detector to sensor 0 frame

// Variables for sensor static calibration (initial bias)
extern uint8_t         calibration_flag[NUM_SENSORS];       
extern uint16_t        calibIdx[NUM_SENSORS];
extern const uint16_t  calibIter;
extern Eigen::MatrixXf magFieldDataOffset;

// Localization Variables
extern float K;                                         // scaling coefficient from Tesla to Gauss
extern int   valrel;                                    // coefficient for norm orientation penalty
extern float x0[NUM_MAGNETS*3];                         // solution vector for the dipole model
extern float x0_dist[NUM_MAGNETS*6+3];                  // solution vector for the dipole model with external disturbance
extern float x0_cyl[NUM_MAGNETS*7];                     // solution vector for the cylinder model
extern float x0_cyl_dist[NUM_MAGNETS*7+3];              // solution vector for the cylinder model with external disturbance

// Variables for LM (lmdif1 or lmder1) optimization in cminpack library
// see also https://github.com/devernay/cminpack
static const int m = NUM_SENSORS*POSE_PARAMS + NUM_MAGNETS;         // number of equations
static const int n = NUM_MAGNETS*POSE_PARAMS;                       // number of parameters
extern float residuals[m];                                          // output residual vector (to be minimized)
extern float Tol;

// for lmdif
static const int Lwa_lmdif = m*n + 5*n + m;                         // working dimension for lmdif
static const int Lwa_lmdif_dist = m * (n+3) + 5 * (n+3) + m;        // working dimension for lmdif when ext disturbance is considered
extern float Wa_lmdif[Lwa_lmdif];                                   // working array for lmdif                    
extern float Wa_lmdif_dist[Lwa_lmdif_dist];                         // working array for lmdif when ext disturbance is considered
extern int   Iwa[n];                                                // working array for lmdif
extern int   Iwa_dist[n+3];                                         // working array for lmdif when ext disturbance is considered

// for lmder
static const int Lwa_lmder = 5*n + m;                               // working dimension for lmder
static const int Lwa_lmder_dist = 5*(n+3) + m;                      // working dimension for lmder when ext disturbance is considered
extern float Wa_lmder[Lwa_lmder];                                   // working array for lmder
extern float Wa_lmder_dist[Lwa_lmder_dist];                         // working array for lmder when ext disturbance is considered
extern float fjac[m*n];                                             // output cost jacobian
extern float fjac_dist[m*(n+3)];                                    // output cost jacobian when ext disturbance is considered
extern int   ipvt[n];                                               // integer output array defining a permutation matrix
extern int   ipvt_dist[n+3];                                        // integer output array defining a permutation matrix when ext disturbance is considered

// struct for cminpack optimization
struct OptimizationData {
    Eigen::MatrixXf sPos;                               // Sensor positions
    Eigen::MatrixXf B_field;                            // Magnetic field data
    int* sensor_2_ignore;                               // List of ignored sensors
    int size;                                           // number of sensors to be ignored (f any)
    uint8_t ValRel;                                     // Regularization parameter
    bool ConsiderDist;                                  // use the external disturbance estimation
};

// struct for magnetic cylinder model computation 
struct varsF {
    float zP;     // zP
    float zM;     // zM
    float dP;     // dP
    float dM;     // dM
    float kP;     // kP
    float kM;     // kM
    float kcP;    // kcP
    float kcM;    // kcM
    float sigmaP; // sigmaP
    float sigmaM; // sigmaM
};

// FUNCTIONS
class LM_library 
{
public:

    // Functions for Streaming, Visualization and Debugging
    void printMagData(Eigen::MatrixXf magData);
    void streamMagTracking(float* vec, float duration);
    void printMagTracking(float* vec);
    void printMagTrackingCyl(float* vec);
    void printMatrix(Eigen::MatrixXf M);
    //void streamMagTracking(float* vec);

    // To (re)initialize random position of the magnets
    void randomize(float* vec);

    // Computaton of the coefficient of determination to evaluate tracking quality
    float determination_coeff(Eigen::MatrixXf res, Eigen::MatrixXf B);

    /*******************************************************************/
    /*General Purpose Linear Algebra Functions*/
    Eigen::MatrixXf Cross_rowwise(Eigen::MatrixXf A, Eigen::MatrixXf B);    // Row-wise cross product
    Eigen::VectorXf Dot_rowwise(Eigen::MatrixXf A, Eigen::MatrixXf B);      // Row-wise dot product
    Eigen::MatrixXf dot_star(Eigen::MatrixXf A, Eigen::MatrixXf B);         // To multiply a Matrix for a Vector
    Eigen::MatrixXf dot_div(Eigen::MatrixXf A, Eigen::MatrixXf B);          // To divide a Matrix for a Vector
    Eigen::MatrixXf scalar_sum(float s, Eigen::MatrixXf A);                 // To sum a Matrix with a Scalar
    Eigen::VectorXf scalar_sum(float s, Eigen::VectorXf A);                 // To sum a Vector with a Scalar
    Eigen::VectorXf Mat2norm(Eigen::MatrixXf A);                            // Row-wise norm operator

    /*******************************************************************/
    /*Dipole Model*/

    // For Huimin
    void localize_magnets_lmder_huimin(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel);
    static int localization_cost_dipole_jac_cmin_huimin(void* p, int m, int n, const float* x_0, float* residuals, float* fjac, int ldfjac, int iflag);
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> localization_cost_dipole_jac_huimin(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, int iflag);
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> dipole_model_jac_huimin(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, int iflag);
    // -----------

    // lmder1 dipole
    // magnetic field model and jacobian function
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> dipole_model_jac(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, bool ConsiderDist, int iflag);
    // computation of the cost function and its jacobian with the dipole model
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> localization_cost_dipole_jac(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist, int iflag);
    // function wrapper for cminpack
    static int localization_cost_dipole_jac_cmin(void* p, int m, int n, const float* x_0, float* residuals, float* fjac, int ldfjac, int iflag);
    // function call to localization (in the main)
    void localize_magnets_lmder(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist);
    
    // lmdif1 dipole
    // magnetic field model function
    Eigen::MatrixXf dipole_model(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos);
    // computation of the cost function with the dipole model
    Eigen::MatrixXf localization_cost_dipole(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist);
    // function wrapper for cminpack
    static int localization_cost_dipole_cmin(void* p, int m, int n, const float* x_0, float* residuals, int iflag);
    // function call to localization (in the main)
    void localize_magnets_lmdif(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, const int size, uint8_t ValRel, bool ConsiderDist);

    /*******************************************************************/
    /*Cylinder Model*/

    // lmder1 cyl analytic
    // magnetic field model and jacobian function
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> cyl_model_jacf(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, int iflag, float BulirschTol, bool ConsiderDist);
    // computation of the cost function and its jacobian with the cylinder model
    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> localization_cost_cyl_jac(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist, int iflag);
    // function wrapper for cminpack
    static int localization_cost_cyl_jac_cmin(void* p, int m, int n, const float* x_0, float* residuals, float* fjac, int ldfjac, int iflag);
    // function call to localization (in the main)
    void localize_magnets_cyl_lmder(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel, bool ConsiderDist);

    // lmdif1 cyl analytic
    // magnetic field model function
    Eigen::MatrixXf cyl_modelf(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, float BulirschTol);
    // computation of the cost function with the cylinder model
    Eigen::MatrixXf localization_cost_cyl(Eigen::MatrixXf x_0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, int size, uint8_t ValRel);
    // function wrapper for cminpack
    static int localization_cost_cyl_cmin(void* p, int m, int n, const float* x_0, float* residuals, int iflag);
    // function call to localization (in the main)
    void localize_magnets_cyl_lmdif(float *x0, Eigen::MatrixXf B_field, Eigen::MatrixXf sPos, int* sensor_2_ignore, const int size, uint8_t ValRel);

    std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> cyl_model_jacf_opt(Eigen::MatrixXf x_0, Eigen::MatrixXf sPos, int iflag, float BulirschTol, bool ConsiderDist);


    // Auxiliary Function for Field and Jacobian computation in Cylinder model
    // see also: https://advanced.onlinelibrary.wiley.com/doi/full/10.1002/advs.202301033
    float BulirschCEL(float kc, float p, float a, float b, float BulirschTol);
    float HeumanLambda(float beta, float k, float BulirschTol);
    float fLambdaLocalization(float rho, float z, float l, float kP, float kM, float sigmaP, float sigmaM, float BulirschTol);
    float fun1Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol);
    float fun2Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol);
    float fun3Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol);
    float fun4Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol);
    float fun5Localization(float rho, float z, float l, struct varsF auxVar, float BulirschTol);

};

// structure to pass parameters and variables to lmder and lmdif
struct OptAndInstance {
    LM_library* instance;
    OptimizationData* optData;
};

#endif