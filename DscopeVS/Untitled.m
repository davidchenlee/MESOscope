clear all

A = importdata('_photon-counts.txt');
plot(A)

for kk = 1:400
    if mod(kk,2)
        B(:,kk) = A(((kk-1)*400)+(1:400));
    else
        B(:,kk) = fliplr( A( ((kk-1)*400)+(1:400) ) );
    end
end