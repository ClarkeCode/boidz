//Header for implementing an entity component system architecture

#ifndef ECS_HEADER
#define ECS_HEADER
#include <vector>
#include <memory>//smart pointers
#include <optional>//std optional
#include <algorithm>//findif
namespace ecs {
    
    template<class PotentialSubclass, class BaseClass>
    inline bool isSubclass(BaseClass* src) {
        return dynamic_cast<PotentialSubclass*>(src) != nullptr;
    }

    class Component {
        public:
        using ptr_t = std::shared_ptr<Component>;
        using ComponentContainer = std::vector<Component::ptr_t>;

        virtual ~Component() = default;
    };

    class Entity {
        public:
        using ptr_t = std::shared_ptr<Entity>;
        using EntityContainer = std::vector<Entity>;
        int id;
        Component::ComponentContainer components;

        template <class DesiredComponentType>
        Component::ptr_t getComponent() const {
            return std::find_if(components.cbegin(), components.cend(), isSubclass<DesiredComponentType, Component>);
        }

        Entity() {}
        Entity(int startingCapacity) { components.reserve(startingCapacity); }
        virtual ~Entity() = default;
    };
    class System {
        friend class Entity;
        public:
        virtual ~System() = default;
        virtual void process(Entity::EntityContainer const& entities) = 0;
    };
}

#endif