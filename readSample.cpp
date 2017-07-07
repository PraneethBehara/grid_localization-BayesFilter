#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Read);

std::vector<std::string> topics;
topics.push_back(std::string("chatter"));
topics.push_back(std::string("numbers"));

rosbag::View view(bag, rosbag::TopicQuery(topics));

foreach(rosbag::MessageInstance const m, view)
{

	if (m.getTopic() == "chatter") 
	{
		std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>(); //std_msgs::String is the type of message within "chatter" topic
		if (s != NULL)
		  // Do whatereve you want with the message (s is a pointer to the message)
			ASSERT_EQ(s->data, std::string("foo"));
	}

	if (m.getTopic() == "numbers") 
	{
		std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>(); //std_msgs::Int32 is the type of message within "numbers" topic
		if (i != NULL)
		  // Do whatereve you want with the message (i is a pointer to the message)
			ASSERT_EQ(i->data, 42);
	}

}

bag.close();
