using System;
using System.Collections.Generic;

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

        public static T Get<T>() where T : class
        {
            if (Services.TryGetValue(typeof(T), out var value))
            {
                return value as T;
            }

            throw new Exception($"Service {typeof(T)} not found.");
        }
    }
}
