#include <opensot_visual_servoing/utils/Utils.h>

opensot_visual_servoing::VisualFeatures opensot_visual_servoing::utils::toVisualFeatureMsg(
    const std::list<vpBasicFeature*>& feature_list,
    const std::string& features_type)
{
    opensot_visual_servoing::VisualFeatures msg;
    opensot_visual_servoing::VisualFeature feature;

    if(features_type == "vpFeaturePoint")
    {
        for(auto f : feature_list)
        {
           vpFeaturePoint* p = (vpFeaturePoint*)(f);
           feature.type = opensot_visual_servoing::VisualFeature::POINT;
           feature.x = p->get_x();
           feature.y = p->get_y();
           feature.Z = p->get_Z(); //
           msg.features.push_back(feature);
        }
    }

    msg.header.stamp = ros::Time::now();

    return msg;
}

std::list<vpBasicFeature*> opensot_visual_servoing::utils::getFeaturesFromMsg(opensot_visual_servoing::VisualFeaturesConstPtr msg)
{
    std::list<vpBasicFeature *> generic_features;

    std::list<vpFeaturePoint *> point_features;
    for(unsigned int i = 0; i < msg->features.size(); ++i)
    {
        opensot_visual_servoing::VisualFeature f = msg->features[i];
        if(f.type == opensot_visual_servoing::VisualFeature::POINT)
        {
            point_features.push_back(new vpFeaturePoint());
            point_features.back()->buildFrom(f.x, f.y, f.Z);
         }
        //else if (f.type == opensot_visual_servoing::VisualFeature::LINE) ...
    }

    for(auto pf : point_features)
        generic_features.push_back(pf);
    return generic_features;
}