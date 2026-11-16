#include <atomic>
#include <string>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/components/DetachableJoint.hh>

using namespace gz;
class ContactSuction final: public sim::System, public sim::ISystemConfigure, public sim::ISystemPreUpdate {
  transport::Node node;
  transport::Node::Publisher status;
  std::atomic<bool> enabled{false};
  sim::Entity joint=sim::kNullEntity, cup=sim::kNullEntity;
  std::string held;
 public:
  void Configure(const sim::Entity &,const std::shared_ptr<const sdf::Element>&,sim::EntityComponentManager&,sim::EventManager&) override {
    node.Subscribe("/suction/enable", &ContactSuction::Enable, this);
    status=node.Advertise<msgs::StringMsg>("/suction/held");
  }
  void Enable(const msgs::Boolean &m) {enabled=m.data();}
  void PreUpdate(const sim::UpdateInfo &info,sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    if(cup==sim::kNullEntity) cup=ecm.EntityByComponents(sim::components::Link(),sim::components::Name("suction_cup"));
    if(!enabled && joint!=sim::kNullEntity) {ecm.RequestRemoveEntity(joint);joint=sim::kNullEntity;held.clear();}
    if(enabled && joint==sim::kNullEntity && cup!=sim::kNullEntity) {
      ecm.Each<sim::components::ContactSensorData,sim::components::ParentEntity>(
        [&](const sim::Entity &,const sim::components::ContactSensorData *data,const sim::components::ParentEntity *parent) {
          if(parent->Data()!=cup) return true;
          for(const auto &contact:data->Data().contact()) {
            if(contact.position_size()==0) continue;
            auto a=contact.collision1().id(),b=contact.collision2().id();
            auto pa=ecm.Component<sim::components::ParentEntity>(a),pb=ecm.Component<sim::components::ParentEntity>(b);
            if(!pa || !pb) continue;
            auto other=pa->Data()==cup?pb->Data():(pb->Data()==cup?pa->Data():sim::kNullEntity);
            if(other==sim::kNullEntity) continue;
            auto mp=ecm.Component<sim::components::ParentEntity>(other);
            if(!mp) continue;
            auto model=mp->Data(); auto name=ecm.Component<sim::components::Name>(model); auto isStatic=ecm.Component<sim::components::Static>(model);
            // Object prefix is an eligibility boundary, never a source of color.
            if(!name || name->Data().rfind("part_",0)!=0 || (isStatic && isStatic->Data())) continue;
            auto d=sim::worldPose(cup,ecm).Pos().Distance(sim::worldPose(other,ecm).Pos());
            if(d>.045) continue;
            joint=ecm.CreateEntity(); ecm.CreateComponent(joint,sim::components::DetachableJoint({cup,other,"fixed"})); held=name->Data(); break;
          }
          return true;
        });
    }
    msgs::StringMsg m;m.set_data(held);status.Publish(m);
  }
};
GZ_ADD_PLUGIN(ContactSuction,sim::System,sim::ISystemConfigure,sim::ISystemPreUpdate)

// Evaluation-only observer: publish current physics transforms even for sleeping
// objects. SceneBroadcaster's change notifications are not a freshness proof.
#include <chrono>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/Utility.hh>
class StateAuditor final: public sim::System, public sim::ISystemConfigure, public sim::ISystemPostUpdate {
  transport::Node node;
  transport::Node::Publisher publisher;
  std::chrono::steady_clock::duration last{};
 public:
  void Configure(const sim::Entity &,const std::shared_ptr<const sdf::Element>&,sim::EntityComponentManager&,sim::EventManager&) override {
    publisher=node.Advertise<msgs::Pose_V>("/evaluation/poses");
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    if(info.paused || info.simTime-last<std::chrono::milliseconds(50))return;
    last=info.simTime;msgs::Pose_V message;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);message.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    ecm.Each<sim::components::Model,sim::components::Name>([&](const sim::Entity &entity,const sim::components::Model *,const sim::components::Name *name){
      if(name->Data().rfind("part_",0)==0){auto p=message.add_pose();msgs::Set(p,sim::worldPose(entity,ecm));p->set_name(name->Data());p->set_id(entity);}return true;
    });
    publisher.Publish(message);
  }
};
GZ_ADD_PLUGIN(StateAuditor,sim::System,sim::ISystemConfigure,sim::ISystemPostUpdate)
