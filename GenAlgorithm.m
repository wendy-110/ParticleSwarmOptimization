function [outAllStrings] = GenAlgorithm(allStrings,aa_array,ba_array,ar_array,br_array,at_array,bt_array)
% This function takes the existing parameters, orders them based on
% performance, and then creates a new set of parameters. 

numStrings = size(allStrings,1);
outAllStrings = zeros(numStrings,size(allStrings,2));

% SortRows
sortedStrings = sortrows(allStrings,9); %Sorting on LOWEST AVERAGE COST

% Mate First Two Sets of Parents
% (i.e. Parents 1+2, Parents 3+4)
for i = 1:2
    mate1 = sortedStrings(2*i-1,1:6);
    mate2 = sortedStrings(2*i,1:6);
    randPar1 = rand();
    randPar2 = rand();
    offspring1 = mate1.*randPar1 + mate2.*(1-randPar1);
    offspring2 = mate1.*randPar2 + mate2.*(1-randPar2);
    outAllStrings(2*i-1,1:6) = offspring1;
    outAllStrings(2*i,1:6) = offspring2;
end

% Generate remaining strings
stringsLeft = numStrings - 4; %4 children made

for i = 5:numStrings
    outAllStrings(i,1) = aa_array(randi(100));
    outAllStrings(i,2) = ba_array(randi(100));
    outAllStrings(i,3) = ar_array(randi(100));
    outAllStrings(i,4) = br_array(randi(100));
    outAllStrings(i,5) = at_array(randi(100));
    outAllStrings(i,6) = bt_array(randi(100));
end
