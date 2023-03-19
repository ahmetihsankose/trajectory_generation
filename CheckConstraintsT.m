function [ Tout ] = CheckConstraintsT( Tin )
amax = 0.999999;
amin = 0.95;
n = length(Tin);
Tout=Tin;
if ( Tout(n-1) < Tout(n) )
    Tout(n) = sqrt(Tout(n-1)*Tout(n));
    Tout(n-1) = Tout(n);
end
for i=n-2:-1:1
          if ( Tout(i)< Tout(i+1) + Tout(i+2) ) 
            a = -Tout(i+2)/2/Tout(i+1)+sqrt((Tout(i+2)/2/Tout(i+1))^2+Tout(i)/Tout(i+1));   
            a = min([max([a amin]), amax]);
            Tout(i) = Tout(i)/a;
            Tout(i+1) = Tout(i+1)*a;
            Tout  = CheckConstraintsT( Tout );
          end
end
end