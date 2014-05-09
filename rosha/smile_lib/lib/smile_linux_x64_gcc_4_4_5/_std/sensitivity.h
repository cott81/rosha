#ifndef DSL_SENSITIVITY_H
#define DSL_SENSITIVITY_H

// {{SMILE_PUBLIC_HEADER}}

#include <vector>
#include <utility>

class DSL_network;
class DSL_Dmatrix;
class DSL_sensRes;


class DSL_sensitivity
{
public:
	DSL_sensitivity();
	~DSL_sensitivity();

	int Calculate(DSL_network &net, bool relevance = true);

	typedef std::pair<int, int> Target; // a target node handle followed by a target outcome index

	double GetMaxSensitivity() const;
	double GetMaxSensitivity(Target target) const;
	double GetMaxSensitivity(int node) const;
	void GetMaxSensitivity(int node, DSL_Dmatrix &maxSens) const;
	double GetMaxSensitivity(int node, Target target) const;

	const DSL_Dmatrix* GetSensitivity(int node, Target target) const;
	
	// GetCoefficients output is a 4-element vector with alpha, beta, gamma, delta
	void GetCoefficients(int node, Target target, std::vector<const DSL_Dmatrix *> &coeffs) const; 

	void GetTargets(std::vector<Target> &targets) const;

private:
	DSL_sensRes *res;
};

#endif
