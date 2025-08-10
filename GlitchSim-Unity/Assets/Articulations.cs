using UnityEngine;

public class AutoArticulationBuilder : MonoBehaviour
{
    public ArticulationJointType defaultJointType = ArticulationJointType.RevoluteJoint;

    void Start()
    {
        foreach (var jointGO in GetComponentsInChildren<Transform>())
        {
            if (jointGO.name.EndsWith("_joint"))
            {
                var ab = jointGO.gameObject.GetComponent<ArticulationBody>();
                if (ab == null)
                    ab = jointGO.gameObject.AddComponent<ArticulationBody>();

                // Set root immovability
                if (jointGO.parent == transform)
                    ab.immovable = true;

                // Set joint type
                ab.jointType = defaultJointType;

                // Optionally use your custom properties here
                // For example: ab.mass = jointGO.gameObject.GetComponent<YourPropHolder>().mass;

                // Optionally set motion constraints & drive
                ab.xDrive = ConfigureDrive();
                ab.yDrive = ConfigureDrive();
                ab.zDrive = ConfigureDrive();
            }
        }
    }

    ArticulationDrive ConfigureDrive()
    {
        return new ArticulationDrive
        {
            stiffness = 1000f,
            damping = 10f,
            forceLimit = 100f,
            lowerLimit = -45f,
            upperLimit = 45f
        };
    }
}
