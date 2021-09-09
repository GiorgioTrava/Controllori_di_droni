clc
clear
for i = 1:5000000
    
    y(i) = randn(1);
   
end
x1 = 1:5000000;
figure()
plot(x1,y)

hist(y,100)