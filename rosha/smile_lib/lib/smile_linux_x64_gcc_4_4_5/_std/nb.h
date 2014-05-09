#ifndef DSL_NB_H
#define DSL_NB_H

// {{SMILE_PUBLIC_HEADER}}

#include <string>
#include <vector>

class DSL_dataset;
class DSL_network;
class DSL_progress;

class DSL_nb
{
public:
    DSL_nb()
    {
		feature_selection = false;
 		priorLinkProbability = 0.001;
		priorSampleSize = 50;
    }

	int Learn(DSL_dataset &ds, DSL_network &net, DSL_progress *progress = NULL) const;

	std::string classvar;
	bool feature_selection;
 	double priorLinkProbability;
	int priorSampleSize;

private:
	int PreChecks(const DSL_dataset &ds_, int &cvar) const;
};

#endif
