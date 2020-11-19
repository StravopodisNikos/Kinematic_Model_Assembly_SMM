function Population = mixed_int_pop_gamult(GenomeLength, ~, options)
% Function that creates an initial population satisfying bounds and
% integer constraints
% Input: IntCon: Vector that specifies integers in chromosome vector
IntCon = [1,2,3,13,14];

totalPopulation = sum(options.PopulationSize);

range = options.PopInitRange;
lower = range(1,:);
span =  range(2,:) - lower;

Population = repmat(lower,totalPopulation,1 )+  ...
    repmat(span,totalPopulation,1) .* rand(totalPopulation, GenomeLength);

x = rand;

if x>=0.5
    Population(:,IntCon) = floor(Population(:, IntCon));
else
    Population(:,IntCon) = ceil(Population(:, IntCon));
end

Population = checkboundsIntGA(Population, range);
end