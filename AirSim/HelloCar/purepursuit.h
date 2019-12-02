#pragma once
class purepursuit
{
public:
	purepursuit(float baseline, float steermax, float lookahead_distance);
	~purepursuit();
};



purepursuit::purepursuit( float baseline,float steermax, float lookahead_distance)
{
	purepursuit data;
	data.baseline = baseline;
	data.steermax = steermax;
	data.lookahead_distance = lookahead_distance;
	data.steering = 0;
	data.velocity = 0;
	data.waypoints = [];
	obj = class (data, 'purpursuit');
}


purepursuit::~purepursuit()
{
}
