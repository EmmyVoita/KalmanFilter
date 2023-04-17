using UnityEngine;
using System.Collections;
using TMPro;
using System.Collections.Generic;

public class Main : MonoBehaviour
{
    //[RequireComponent(typeof(Rigidbody))]
   


    [Header("References")]
    public TextMeshProUGUI textMeshProSpeed;
    public TextMeshProUGUI textMeshProFilteredSpeed;
    public GraphController controller;
    public RectTransform windowReference;
    public CarController carController;
    public Rigidbody rb;
    public BoxCollider collider;
    private KalmanFilter kalmanFilter;
    public WheelCollider wheelCollider;

    private List<float> unfilteredSpeedPoints = new List<float>();
    private List<float> filteredSpeedPoints = new List<float>();
    private List<float> realSpeedPoints = new List<float>();

    [Header("Variables")]
    public float sensorNoiseFactor = 0.1f;
    public float sensorUpdateInterval = 0.1f; // update interval in seconds
    private float timer = 0.0f;
    [Range(0f, 100f)]
    public float measurementNoiseScalar = 0.1f;
    [Range(0f, 20f)]
    public float processNoiseScalar = 0.0f;
    public bool useControlMatrix = false;
    public Vector3 controlVector;
    public float acceleration;
    public float dragForce;
    public float airDensity = 1.2f;
    public float rollingResistance;




    private void Start()
    {
        // Instantiate Kalman filter
        kalmanFilter = new KalmanFilter();

        // transition matrix would be a 3x3 identity matrix, because the state I
        // am trying to estimate is the same as the observation that I am making
        Matrix3x3 F = Matrix3x3.identity;

        // since i am controlling the car I can use a control matrix to impelement
        // the control inputs into the state update equation
        Matrix3x3 B = new Matrix3x3(sensorUpdateInterval * Mathf.Cos(carController.currentSteerAngle), 0, 1,
                                    sensorUpdateInterval * Mathf.Sin(carController.currentSteerAngle), 0, 0,
                                    0, sensorUpdateInterval, 0);

        Matrix3x3 Q = new Matrix3x3(1f, 0, 0,
                                    0, 1f, 0,
                                    0, 0, 1f);
        Matrix3x3 H = Matrix3x3.identity;
        Matrix3x3 R = new Matrix3x3(measurementNoiseScalar * sensorNoiseFactor, 0, 0,
                                    0, measurementNoiseScalar * sensorNoiseFactor, 0,
                                    0, 0, measurementNoiseScalar * sensorNoiseFactor);
   

        //Set model parameters
        kalmanFilter.SetStateTransition(F);
        kalmanFilter.SetControlMatrix(B);
        kalmanFilter.SetProcessNoise(Q);
        //kalmanFilter.SetMeasurementMatrix(H);
        kalmanFilter.SetMeasurementNoise(R);
    }

    private void Update()
    {

        timer += Time.deltaTime; // increment the timer by the elapsed time since last frame

        if (timer >= sensorUpdateInterval) // check if it's time to update the sensor data
        {
            // Get velocity data
            Vector3 velocity = GetVelocityData();

            UpdateKalmanInputs(velocity);



            // Perform prediction step
            kalmanFilter.Predict();

            // Perform update step with velocity measurement
            kalmanFilter.Update(velocity);

            // Get filtered velocity estimate
            Vector3 filteredVelocity = kalmanFilter.GetState();

            // Use filtered velocity estimate in your code
            //...
            textMeshProSpeed.SetText("Unfiltered Speed: " + velocity.magnitude);
            textMeshProFilteredSpeed.SetText("Filtered Speed: " + filteredVelocity.magnitude);

            if (unfilteredSpeedPoints.Count == 39)
            {
                unfilteredSpeedPoints.RemoveAt(0);
            }
            unfilteredSpeedPoints.Add(velocity.magnitude);

            if (filteredSpeedPoints.Count == 39)
            {
                filteredSpeedPoints.RemoveAt(0);
            }
            filteredSpeedPoints.Add(filteredVelocity.magnitude);

            if (realSpeedPoints.Count == 39)
            {
                realSpeedPoints.RemoveAt(0);
            }
            realSpeedPoints.Add(rb.velocity.magnitude);

            if (windowReference.gameObject.activeSelf)
            {
                controller.ShowGraph(unfilteredSpeedPoints, filteredSpeedPoints, realSpeedPoints);
            }
           

            timer = 0.0f;
        }

        if (Input.GetKeyDown(KeyCode.Tab) && windowReference.gameObject.activeSelf)
        {
            windowReference.gameObject.SetActive(false);
        }
        else if (Input.GetKeyDown(KeyCode.Tab))
        {
            windowReference.gameObject.SetActive(true);
        }
    
    }


    private void UpdateKalmanInputs(Vector3 velocity)
    {
        Matrix3x3 R = new Matrix3x3(measurementNoiseScalar * sensorNoiseFactor, 0, 0,
                                  0, measurementNoiseScalar * sensorNoiseFactor, 0,
                                  0, 0, measurementNoiseScalar * sensorNoiseFactor);

        Matrix3x3 Q = new Matrix3x3(processNoiseScalar, 0, 0,
                                    0, processNoiseScalar, 0,
                                    0, 0, processNoiseScalar);

        GetControl();
        kalmanFilter.SetMeasurementNoise(R);
        kalmanFilter.SetProcessNoise(Q);
    }

    private Vector3 GetVelocityData()
    {
        // Return velocity data from sensor or other source
        Vector3 velocity = rb.velocity + new Vector3(UnityEngine.Random.Range(-1f, 1f) * sensorNoiseFactor, UnityEngine.Random.Range(-1f, 1f) * sensorNoiseFactor, UnityEngine.Random.Range(-1f, 1f) * sensorNoiseFactor);
        return velocity;
    }

    public void GetControl()
    {

        float width = collider.size.x;
        float height = collider.size.y;
        float depth = collider.size.z;
        float A = 2 * (width * height + width * depth + height * depth);

        float wheelRadius = wheelCollider.radius;
        float wheelWidth = wheelRadius * 2f * Mathf.PI / wheelCollider.forwardFriction.extremumSlip;
        float friction = wheelCollider.forwardFriction.extremumValue;
        float slip = wheelCollider.forwardFriction.extremumSlip;
        float rollingResistanceCoefficient = friction * wheelRadius / (wheelWidth * slip);


        

        dragForce = 0.5f * airDensity * rb.drag * Mathf.Pow(GetVelocityData().magnitude, 2) * A;
        rollingResistance = rollingResistanceCoefficient * rb.mass;
        dragForce += rollingResistance;

        if (Mathf.Abs(rb.velocity.magnitude) < 0.1f)
        {
            acceleration = 0f;
        }
        else
        {
            acceleration = (carController.motorTorque - dragForce) / rb.mass;
        }
        //acceleration = (carController.motorTorque - dragForce) / rb.mass;
        //float angularAcceleration = acceleration * Mathf.Tan(carController.currentSteerAngle) / carController.wheelBase;


        // local space vector
        controlVector = new Vector3(sensorUpdateInterval * Mathf.Cos(carController.currentSteerAngle) * acceleration,
                                    0,
                                    sensorUpdateInterval * Mathf.Sin(carController.currentSteerAngle) * acceleration);
        // world space vector
        controlVector = gameObject.transform.TransformVector(controlVector);


        Matrix3x3 B;
        if (useControlMatrix)
        {
            B = new Matrix3x3(1, 0, 0,
                              0, 1, 0,
                              0, 0, 1);
        }
        else
        {
            B = new Matrix3x3(0, 0, 0,
                            0, 0, 0,
                            0, 0, 0);
        }

        kalmanFilter.SetControlVector(controlVector);
        kalmanFilter.SetControlMatrix(B);
    }
}
