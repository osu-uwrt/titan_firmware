#pragma once

#include <memory>
#include <unordered_map>

namespace Canmore {

template <typename T, typename ConstructorKey, typename ConstructorKeyHash>
class SocketSingleton
{
    public:
        template <typename... ConstructorArgs>
        static std::shared_ptr<T> create(ConstructorArgs&&... _args) {
            static std::unordered_map<ConstructorKey,std::weak_ptr<T>,ConstructorKeyHash> instances;

            // Search for instance
            auto key = ConstructorKey(std::forward<ConstructorArgs>(_args)...);
            auto it = instances.find(key);
            if (it != instances.end()) {
                // Weak pointer exists, check if still valid
                auto inst = it->second.lock();
                if (inst) {
                    // Client still valid, we can return it
                    return inst;
                }
            }

            // We couldn't get a client, make a new one
            auto inst = std::shared_ptr<T>(new T(std::forward<ConstructorArgs>(_args)...));
            instances[key] = inst;
            return inst;
        }

        SocketSingleton(SocketSingleton const &) = delete;
        SocketSingleton& operator=(SocketSingleton const &) = delete;

    protected:
        SocketSingleton() {}
};

};