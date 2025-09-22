using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace GlitchSim.Runtime.Core
{
    /// <summary>
    /// Provides a mechanism to retrieve common services while avoiding tight coupling.
    /// </summary>
    public static class ServiceLocator
    {
        private static readonly Dictionary<Type, object> Services = new();

        public static void Register<T>(T service) where T : class
        {
            Services[typeof(T)] = service;
        }

        public static void Unregister<T>() where T : class
        {
            Services.Remove(typeof(T));
        }

        public static bool TryGet<T>(out T service) where T : class
        {
            if (Services.TryGetValue(typeof(T), out var value))
            {
                service = value as T;
                return true;
            }

            service = null;
            return false;
        }

        public static IEnumerator TryGet<T>(Action<T> callback, float timeoutSeconds) where T : class
        {
            T service = null;
            
            while (!TryGet(out service) && timeoutSeconds > 0)
            {
                yield return null;
                timeoutSeconds -= Time.deltaTime;
            }

            // call back whether we got the service or not so users can handle failure as well as success
            callback(service);
        }
    }
}
