function [ synchDataFast] = synchronizeData( dataSlow, dataFast )
%SYNCHRONIZEDATA Synchronize two data matrixes sampled at different rates.
%   Given 2 matrixes of data, where the first column contains the
%   timestamps, synchronize the two. The i-th element of synchDataFast is 
%   the element of dataFast with the timestamp that is the closest to the 
%   timestamp of the i-th element of dataSlow.
    synchDataFast = zeros(size(dataSlow,1), size(dataFast,2));
    j=1;
    for i=1:size(dataSlow,1)
       timeSlow = dataSlow(i,1);       
       while dataFast(j,1)<timeSlow && j<size(dataFast,1)
           j=j+1;
       end
       if(j>1)
           % if current dataSlow element is closer to dataFast element j than
           % j-1
           if(abs(timeSlow-dataFast(j,1))<abs(timeSlow-dataFast(j-1,1)))
               synchDataFast(i,:) = dataFast(j,:);
           else
               synchDataFast(i,:) = dataFast(j-1,:);
           end
           j=j-1;
       else
           synchDataFast(i,:) = dataFast(1,:);
       end
    end
end

