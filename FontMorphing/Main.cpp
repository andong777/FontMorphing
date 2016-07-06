#include "FontMorphing.h"
#include "Utility.h"
#include <ctime>

// Ä¬ÈÏ²ÎÊý
string sourceCharDir = "TestData\\PKU\\HT";
string targetCharDir = "TestData\\PKU\\LS";
string charNameOrList = "FM1";
string outputCharDir = "TestData\\output";
float ratio = 0.5;
bool display = true;
time_t tstart, tend;
int main(int argc, char** argv)
{

	if (argc >= 7){
		sourceCharDir = argv[1];
		targetCharDir = argv[2];
		charNameOrList = argv[3];
		outputCharDir = argv[4];
		ratio = atof(argv[5]);
		display = *argv[6] == 'y' || *argv[6] == 'Y';
		cout << "use user setting: " << endl;
	}
	else{
		cout << "use default demo data: " << endl;
	}
	cout << "source character directory: " << sourceCharDir << endl;
	cout << "target character directory: " << targetCharDir << endl;
	cout << "character name or list: " << charNameOrList << endl;
	cout << "output character directory: " << outputCharDir << endl;
	cout << "morphing ratio: " << ratio << endl;
	cout << "display: " << display << endl;

	if (ratio <= 0) {
		cout << "invalid parameters!" << endl;
		return -1;
	}

	tstart = time(0);
	cout << endl << "*** processing character " << charNameOrList << "... ***" << endl << endl;
	vector<string> charList;

	if (has_suffix(charNameOrList, ".txt")) {	// a list
		ifstream f(charNameOrList);
		if (f.is_open()){
			string thisCharName;
			while (f >> thisCharName){
				charList.push_back(thisCharName);
			}
			f.close();
		}
		else{
			cout << "character list not found! Exit" << endl;
			return -1;
		}
	}
	else {	// a single name
		charList.push_back(charNameOrList);
	}

#ifdef PARALLEL_MODE
//#pragma omp parallel for num_threads(4)
#endif
	for (int i = 0; i < charList.size(); i++){
		string charName = charList[i];
		cout << endl << "*** processing character " << charName << "... ***" << endl << endl;
		FontMorphing *morphing = new FontMorphing(charName);
		try {
			bool done = morphing->work(ratio, display);
			if (done){
				cout << "*** character " << charName << "done ***" << endl;
			}
			else{
				cout << "!!! character " << charName << "failed !!!" << endl;
			}
		} catch (...) {
			std::cerr << "Unknown failure occured. Possible memory corruption" << std::endl;
		}
		delete morphing;
	}

	tend = time(0);
	ofstream of("time.txt");
	int seconds = difftime(tend, tstart);
	int hours = seconds / 3600;
	seconds %= 3600;
	int minutes = seconds / 60;
	seconds %= 60;
	of << "" << hours + " h " << minutes << " m " << seconds << " s." << endl;
	of.close();
	return 0;
}