package org.usfirst.frc.team1257.util;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Ultrasonic;

public class StabilizedUltrasonic implements PIDSource
{
	private Ultrasonic m_distanceSensor;
	static final int MAX_NUM_OF_DISTANCES = 11;
	LinkedList<Double> m_prevDistancesInches;
	LinkedList<Double> m_prevDistancesMM;
	
	public StabilizedUltrasonic(int pingChannel, int echoChannel)
	{
		m_distanceSensor = new Ultrasonic(pingChannel, echoChannel);
		m_prevDistancesInches = new LinkedList<Double>();
		m_prevDistancesMM = new LinkedList<Double>();
	}
	
	double average(LinkedList<Double> list)
	{
		double sum = 0;
		for(int i = 0; i < list.size(); i++)
		{
			sum += list.get(i);
		}
		
		return sum / list.size();
	}
	
	double median(LinkedList<Double> list)
	{
		int midPoint = list.size() / 2;
		return list.get(midPoint);
	}
	
	void update()
	{
		if(m_prevDistancesInches.size() == MAX_NUM_OF_DISTANCES)
		{
			m_prevDistancesInches.removeFirst();
		}
		m_prevDistancesInches.add(m_distanceSensor.getRangeInches());
		
		if(m_prevDistancesMM.size() == MAX_NUM_OF_DISTANCES)
		{
			m_prevDistancesMM.removeFirst();
		}
		m_prevDistancesMM.add(m_distanceSensor.getRangeMM());
	}
	
	double getRangeInches()
	{
		return median(m_prevDistancesInches);
	}
	
	double getRangeMM()
	{
		return median(m_prevDistancesMM);
	}

	public double pidGet() 
	{
		return getRangeInches();
	}

	public void setPIDSourceType(PIDSourceType pidSource) 
	{
		
	}

	public PIDSourceType getPIDSourceType()
	{
		return PIDSourceType.kDisplacement;
	}
}
