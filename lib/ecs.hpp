//Header for implementing an entity component system architecture

#ifndef ECS_HEADER
#define ECS_HEADER
#include <vector>

namespace ecs {
    
    template<class PotentialSubclass, class BaseClass>
    inline bool isSubclass(BaseClass* src) {
        return dynamic_cast<PotentialSubclass*>(src) != nullptr;
    }

    class Component {
        public:
        using ptr_t = Component*;
        using ComponentContainer = std::vector<Component::ptr_t>;

        virtual ~Component() = default;
    };

    class Entity {
        public:
        using ptr_t = Entity*;
        using EntityContainer = std::vector<Entity>;
        int id;
        Component::ComponentContainer components;

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