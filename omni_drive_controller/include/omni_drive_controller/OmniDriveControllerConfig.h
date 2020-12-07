#ifndef __omni_drive_controller__OMNIDRIVECONTROLLERCONFIG_H__
#define __omni_drive_controller__OMNIDRIVECONTROLLERCONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace omni_drive_controller
{
  class OmniDriveControllerConfigStatics;

  class OmniDriveControllerConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }

      virtual void clamp(OmniDriveControllerConfig &config, const OmniDriveControllerConfig &max, const OmniDriveControllerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const OmniDriveControllerConfig &config1, const OmniDriveControllerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, OmniDriveControllerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const OmniDriveControllerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, OmniDriveControllerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const OmniDriveControllerConfig &config) const = 0;
      virtual void getValue(const OmniDriveControllerConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T OmniDriveControllerConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T OmniDriveControllerConfig::* field;

      virtual void clamp(OmniDriveControllerConfig &config, const OmniDriveControllerConfig &max, const OmniDriveControllerConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const OmniDriveControllerConfig &config1, const OmniDriveControllerConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, OmniDriveControllerConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const OmniDriveControllerConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, OmniDriveControllerConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const OmniDriveControllerConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const OmniDriveControllerConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, OmniDriveControllerConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, OmniDriveControllerConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<OmniDriveControllerConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(OmniDriveControllerConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("left_wheel_radius_multiplier"==(*_i)->name){left_wheel_radius_multiplier = boost::any_cast<double>(val);}
        if("right_wheel_radius_multiplier"==(*_i)->name){right_wheel_radius_multiplier = boost::any_cast<double>(val);}
        if("back_wheel_radius_multiplier"==(*_i)->name){right_wheel_radius_multiplier = boost::any_cast<double>(val);}
        if("wheel_separation_multiplier"==(*_i)->name){wheel_separation_multiplier = boost::any_cast<double>(val);}
        if("publish_rate"==(*_i)->name){publish_rate = boost::any_cast<double>(val);}
        if("enable_odom_tf"==(*_i)->name){enable_odom_tf = boost::any_cast<bool>(val);}
      }
    }

    double left_wheel_radius_multiplier;
double right_wheel_radius_multiplier;
double back_wheel_radius_multiplier;
double wheel_separation_multiplier;
double publish_rate;
bool enable_odom_tf;

    bool state;
    std::string name;

    
}groups;



//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double left_wheel_radius_multiplier;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double right_wheel_radius_multiplier;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double back_wheel_radius_multiplier;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double wheel_separation_multiplier;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      double publish_rate;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      bool enable_odom_tf;
//#line 228 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("OmniDriveControllerConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const OmniDriveControllerConfig &__max__ = __getMax__();
      const OmniDriveControllerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const OmniDriveControllerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const OmniDriveControllerConfig &__getDefault__();
    static const OmniDriveControllerConfig &__getMax__();
    static const OmniDriveControllerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const OmniDriveControllerConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void OmniDriveControllerConfig::ParamDescription<std::string>::clamp(OmniDriveControllerConfig &config, const OmniDriveControllerConfig &max, const OmniDriveControllerConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class OmniDriveControllerConfigStatics
  {
    friend class OmniDriveControllerConfig;

    OmniDriveControllerConfigStatics()
    {
OmniDriveControllerConfig::GroupDescription<OmniDriveControllerConfig::DEFAULT, OmniDriveControllerConfig> Default("Default", "", 0, 0, true, &OmniDriveControllerConfig::groups);
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.left_wheel_radius_multiplier = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.left_wheel_radius_multiplier = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.left_wheel_radius_multiplier = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("left_wheel_radius_multiplier", "double", 0, "Left wheel radius multiplier.", "", &OmniDriveControllerConfig::left_wheel_radius_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("left_wheel_radius_multiplier", "double", 0, "Left wheel radius multiplier.", "", &OmniDriveControllerConfig::left_wheel_radius_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.right_wheel_radius_multiplier = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.right_wheel_radius_multiplier = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.right_wheel_radius_multiplier = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("right_wheel_radius_multiplier", "double", 0, "Right wheel radius multiplier.", "", &OmniDriveControllerConfig::right_wheel_radius_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("right_wheel_radius_multiplier", "double", 0, "Right wheel radius multiplier.", "", &OmniDriveControllerConfig::right_wheel_radius_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.back_wheel_radius_multiplier = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.back_wheel_radius_multiplier = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.back_wheel_radius_multiplier = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("back_wheel_radius_multiplier", "double", 0, "Back wheel radius multiplier.", "", &OmniDriveControllerConfig::back_wheel_radius_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("back_wheel_radius_multiplier", "double", 0, "Back wheel radius multiplier.", "", &OmniDriveControllerConfig::back_wheel_radius_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.wheel_separation_multiplier = 0.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.wheel_separation_multiplier = 1.5;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.wheel_separation_multiplier = 1.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("wheel_separation_multiplier", "double", 0, "Wheel separation multiplier.", "", &OmniDriveControllerConfig::wheel_separation_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("wheel_separation_multiplier", "double", 0, "Wheel separation multiplier.", "", &OmniDriveControllerConfig::wheel_separation_multiplier)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.publish_rate = 0.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.publish_rate = 2000.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.publish_rate = 50.0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("publish_rate", "double", 0, "Publish rate of odom.", "", &OmniDriveControllerConfig::publish_rate)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<double>("publish_rate", "double", 0, "Publish rate of odom.", "", &OmniDriveControllerConfig::publish_rate)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __min__.enable_odom_tf = 0;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __max__.enable_odom_tf = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __default__.enable_odom_tf = 1;
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.abstract_parameters.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<bool>("enable_odom_tf", "bool", 0, "Publish odom frame to tf.", "", &OmniDriveControllerConfig::enable_odom_tf)));
//#line 291 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __param_descriptions__.push_back(OmniDriveControllerConfig::AbstractParamDescriptionConstPtr(new OmniDriveControllerConfig::ParamDescription<bool>("enable_odom_tf", "bool", 0, "Publish odom frame to tf.", "", &OmniDriveControllerConfig::enable_odom_tf)));
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      Default.convertParams();
//#line 246 "/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py"
      __group_descriptions__.push_back(OmniDriveControllerConfig::AbstractGroupDescriptionConstPtr(new OmniDriveControllerConfig::GroupDescription<OmniDriveControllerConfig::DEFAULT, OmniDriveControllerConfig>(Default)));
//#line 366 "/opt/ros/melodic/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"

      for (std::vector<OmniDriveControllerConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<OmniDriveControllerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<OmniDriveControllerConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    OmniDriveControllerConfig __max__;
    OmniDriveControllerConfig __min__;
    OmniDriveControllerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const OmniDriveControllerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static OmniDriveControllerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &OmniDriveControllerConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const OmniDriveControllerConfig &OmniDriveControllerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const OmniDriveControllerConfig &OmniDriveControllerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const OmniDriveControllerConfig &OmniDriveControllerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<OmniDriveControllerConfig::AbstractParamDescriptionConstPtr> &OmniDriveControllerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<OmniDriveControllerConfig::AbstractGroupDescriptionConstPtr> &OmniDriveControllerConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const OmniDriveControllerConfigStatics *OmniDriveControllerConfig::__get_statics__()
  {
    const static OmniDriveControllerConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = OmniDriveControllerConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __OmniDRIVECONTROLLERRECONFIGURATOR_H__
