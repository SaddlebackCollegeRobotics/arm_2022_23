using UnityEngine;

public class CopyTransform : MonoBehaviour
{
    [SerializeField] private Transform target;

    void Start()
    {
        
    }

    void Update()
    {
        transform.position = target.position;
        transform.rotation = target.rotation;
    }
}
