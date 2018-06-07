#ifndef ENHANCED_DASHBOARD
#define ENHANCED_DASHBOARD

#include <WPILib.h>
#include <string>
#include <map>
#include <algorithm>

using namespace std;

class EnhancedDashboard : SmartDashboard
{
private:
	static map<string, void (*)()> buttons;
public:
	static void PutNumber(string keyName, double initialNumber = 0.0)
	{
		SmartDashboard::PutNumber(keyName, initialNumber);
	}

	static void PutString(string keyName, string initialString = "")
	{
		SmartDashboard::PutString(keyName, initialString);
	}

	static void PutBoolean(string keyName, bool initialBool = false)
	{
		SmartDashboard::PutBoolean(keyName, initialBool);
	}

	static double GetNumber(string keyName, double defaultValue = 0.0)
	{
		return SmartDashboard::GetNumber(keyName, defaultValue);
	}

	static string GetString(string keyName, string defaultString = "")
	{
		return SmartDashboard::GetString(keyName, defaultString);
	}

	static bool GetBoolean(string keyName, bool defaultBool = false)
	{
		return SmartDashboard::GetBoolean(keyName, defaultBool);
	}


	static void AddButton(string buttonName, void (*f)())
	{
		if(find(buttons.begin(), buttons.end(), buttonName) != buttons.end())
		{
			buttons.insert(pair<string, void (*)()>(buttonName, f));
			PutBoolean(buttonName);
		}
	}

	static void UpdateButtons()
	{
		auto iterator = buttons.begin();
		while(iterator != buttons.end())
		{
			string buttonName = iterator->first();
			if(GetBoolean(buttonName))
			{
				PutBoolean(buttonName, false);
				(iterator->second())();
			}

			iterator++;
		}
	}
};

#endif
