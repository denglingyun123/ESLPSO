function r = generateR(popsize, i)  
% Generate index for DE mutation 
% Output: r = [r1 r2 r3]

r1 = randi(popsize);
while r1 == i
    r1 = randi(popsize); 
end
 
r2 = randi(popsize);
while r2 == r1 || r2 == i
    r2 = randi(popsize); 
end

r3 = randi(popsize);
while r3 == r2 || r3 == r1 || r3 == i
    r3 = randi(popsize); 
end  

r = [r1  r2  r3];

end