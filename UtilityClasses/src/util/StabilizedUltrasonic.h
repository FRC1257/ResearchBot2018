#ifndef STABILIZED_ULTRASONIC
#define STABILIZED_ULTRASONIC

#include <wpilib.h>

class StabilizedUltrasonic : PIDSource
{
private:
	Ultrasonic DistanceSensor;
	static constexpr int MAX_NUM_OF_DISTANCES = 11;
	std::deque<double> m_prevDistancesInches;
	std::deque<double> m_prevDistancesMM;

public:
	StabilizedUltrasonic(int pingChannel, int echoChannel) :
		DistanceSensor(pingChannel, echoChannel)
	{

	}

	virtual ~StabilizedUltrasonic()
	{

	}

	double average(std::deque<double>& array)
	{
		double sum = 0;
		for(unsigned int i = 0; i < array.size(); i++)
		{
			sum += array[i];
		}
		return sum / array.size();
	}

	double median(std::deque<double> array)
	{
		// Can't use a reference because 'nth_element' would reorder the
		// original array
		size_t midPoint = array.size() / 2;
		nth_element(array.begin(), array.begin() + midPoint, array.end());
		return array[midPoint];
	}

	double PIDGet()
	{
		return GetRangeInches();
	}

	void Update()
	{
		// Delete the oldest measurement when the array of old distances
		// reaches max capacity
		if(m_prevDistancesInches.size() == MAX_NUM_OF_DISTANCES)
		{
			m_prevDistancesInches.pop_front();
		}
		m_prevDistancesInches.push_back(DistanceSensor.GetRangeInches());

		if(m_prevDistancesMM.size() == MAX_NUM_OF_DISTANCES)
		{
			m_prevDistancesMM.pop_front();
		}
		m_prevDistancesMM.push_back(DistanceSensor.GetRangeMM());
	}

	// Return the median of the past few distance values
	double GetRangeInches()
	{
		return median(m_prevDistancesInches);
	}

	double GetRangeMM()
	{
		return median(m_prevDistancesMM);
	}
};

#endif
