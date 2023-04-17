
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KalmanFilter
{
    // State variables
    private Vector3 state;
    private Matrix3x3 covariance;

    // System model
    private Matrix3x3 stateTransition;
    private Matrix3x3 controlMatrix;
    private Vector3 controlVector;
    private Matrix3x3 processNoise;

    // Measurement model
    private float measurementMatrix;
    private Matrix3x3 measurementNoise;

    // Constructor
    public KalmanFilter()
    {
        // Initialize state variables
        state = Vector3.zero;
        covariance = Matrix3x3.identity;

        // Initialize system model
        stateTransition = Matrix3x3.identity;
        controlMatrix = new Matrix3x3(0, 0, 1,
                                      0, 0, 0,
                                      0, 0, 0);
        controlVector = Vector3.zero;
        processNoise = Matrix3x3.identity;

        // Initialize measurement model
        measurementMatrix = 1.0f;
        measurementNoise = Matrix3x3.identity;
    }

    // Prediction step
    public void Predict()
    {

        // Update State:
        // this is the predicition. 

        state = stateTransition * state + controlMatrix * controlVector;

        // Update Covariance:
        // covariance represents the uncertainty or error in the estimated state of the system

        covariance = stateTransition * covariance * stateTransition.Transpose() + processNoise;
    }

    // Update step
    public void Update(Vector3 measurement)
    {
        // Kalman Gain:
        // The Kalman gain determines how much of the new measurement information
        // should be used to update the predicted state estimate. It is used to weigh
        // the relative contributions of the predicted state estimate and the observed measurements.

        // MeasurementNoise:
        // describes the amount of noise present in the measurements provided by the sensor.
        // The more noise, the less the measurement is used to update the state.

        Matrix3x3 kalmanGain = covariance * measurementMatrix *
                              (covariance * measurementMatrix * measurementMatrix + measurementNoise).Invert();

        Debug.Log("KalmanGain:" + kalmanGain.ToString());

        // State Vector:
        // Since the only state variable is a vector3 velocity, the state vector
        // would simply be a vector3 that holds the velocity in each direction.

        // MeasurementMatrix:
        // Since the state variable is only velocity, the measurement matrix would
        // be a row vector that maps the velocity to the measurement space.

        // MeasurementSpace: 
        // the set of all possible values that a measurement can take.
        // Since our sensor can measure any range of values, and the only state
        // variable is velocity, the measureMatrix should be [1.0]

        // Therefore the vector3 multiplied by the kalmanGain would be the
        // measurement velocity minus the previous state

        // [velocity.x]                [state.x]
        // [velocity.y]  -  [1.0]  *   [state.y]
        // [velocity.z]                [state.z]

        Vector3 kalaman = kalmanGain * (measurement - measurementMatrix * state);

        Debug.Log(measurement - measurementMatrix * state);


        // Update state
        state = state + (kalmanGain * (measurement - measurementMatrix * state));

        // Update covariance
        covariance = (Matrix3x3.identity - kalmanGain * measurementMatrix) * covariance;
        
    }

    // Setters for model parameters
    public void SetStateTransition(Matrix3x3 matrix)
    {
        stateTransition = matrix;
    }

    public void SetControlMatrix(Matrix3x3 matrix)
    {
        controlMatrix = matrix;
    }

    public void SetControlVector(Vector3 vector)
    {
        controlVector = vector;
    }

    public void SetProcessNoise(Matrix3x3 matrix)
    {
        processNoise = matrix;
    }

    public void SetMeasurementMatrix(float f)
    {
        measurementMatrix = f;
    }

    public void SetMeasurementNoise(Matrix3x3 matrix)
    {
        measurementNoise = matrix;
    }

    // Getter for state estimate
    public Vector3 GetState()
    {
        return state;
    }
}


/*Matrix3x3 kalmanGain = covariance * measurementMatrix.Transpose() *
                          (measurementMatrix * covariance * measurementMatrix.Transpose() +
                           measurementNoise).Invert();*/