#include <map>
#include "gazebo.hh"
#include "common/common.hh"
#include "physics/physics.hh"

namespace gazebo
{
class AnimateJoints : public ModelPlugin
{
public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
  {
    std::map<std::string, common::NumericAnimationPtr> anim;

    // Create a new animation for the "my_joint" define in the SDF file.
    // The animation will last for 5.0 seconds, and it will repeat
    anim["box::my_joint"].reset(new common::NumericAnimation(
                                                             "my_animation", 5.0, true));

    // Create a key frame for the starting position of the joint
    common::NumericKeyFrame *key = anim["box::my_joint"]->CreateKeyFrame(0.0);
    key->SetValue(0.0);

    // Create a key frame half-way through the animation
    key = anim["box::my_joint"]->CreateKeyFrame(2.5);
    key->SetValue(-3.14);

    // Create the end key frame to be at the same position as the start
    // for a smooth animation
    key = anim["box::my_joint"]->CreateKeyFrame(5.0);
    key->SetValue(0.0);

    // Attach the animation to the model
    _model->SetJointAnimation(anim);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AnimateJoints)
}
