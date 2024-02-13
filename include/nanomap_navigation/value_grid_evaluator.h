#ifndef VALUE_GRID_EVALUATOR_H
#define VALUE_GRID_EVALUATOR_H

#include "motion.h"
#include "value_grid.h"

class ValueGridEvaluator {
public:
	ValueGrid* GetValueGridPtr();

private:
	ValueGrid value_grid;
};

#endif