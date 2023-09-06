data = csvread('Velocidades.csv');
n_regions=64;

[idx,C1] = LBG(data,n_regions);
subplot(2,1,1);
gscatter(data(:,1),data(:,2),idx)
hold on
plot(C1(:,1),C1(:,2),'kx')
title('LBG algorithm')

[idx,C2] = kmeans(data,n_regions);
subplot(2,1,2);
gscatter(data(:,1),data(:,2),idx)
hold on
plot(C2(:,1),C2(:,2),'kx')
title('k means algorithm')


%% First part functions
function [cx,cy] = get_centroid(X) %%This function get the centroid of each region
    m=size(X); %%get the array size
    cx=0;
    cy=0;
    n=m(1);
    for i = 1:n
        cx=cx+X(i,1); %%realice the sum of data
        cy=cy+X(i,2);
    end
    cx=cx/n; %%divide the sum before calculated
    cy=cy/n;
end
%%
function[idX,C]=divide(X,C,nc,k)
    m=size(X);
    n=m(1); %% get the size
    m=m(2);
    d=zeros(nc,1);
    d_ant=0; %% distance variables inicialization
    d_act=1000000000000000;
    count=0;
    idX=zeros(n,1); %%inde inicialization
    while(abs(d_ant-d_act)>0.001 && count <= 400) %%while the tolance is greater than 0.01
        count=count+1; %increase the value of count
        d_ant=d_act;
        d_act=0;
        N=zeros(k,1);
        C_next=zeros(k,m); %%the next centroids calculation
        for i=1:n
            for j=1:nc
                d(j)=sqrt( (C(j,1)-X(i,1) )^2+ (C(j,2)-X(i,2) )^2); 
                % distance between centroid before calculated and data
                % value in i position
            end
            [d_min,j_min]=min(d); %% search for the centroid with the shortest distance
            d_act=d_act+d_min; %%sum of distance for every region
            C_next(j_min,1)=C_next(j_min,1)+X(i,1); %% sum of data in each region
            C_next(j_min,2)=C_next(j_min,2)+X(i,2);
            N(j_min)=N(j_min)+1; %%increase the number of data in each region
            idX(i)=j_min; %%% the region for each data
        end
        for j=1:nc
            C(j,1)=C_next(j,1)/N(j); %%centroid calculation
            C(j,2)=C_next(j,2)/N(j);
        end
    end
    %%count
end
%%
function[idX,C]=LBG(X,k)
    m=size(X);
    data_size=m(1);
    m=m(2);
    idX=ones(data_size,1);
    C=zeros(k,m);
    nc=1;
    [C(1,1),C(1,2)]=get_centroid(X);%%get the first centroid
    while(nc<k)
        for i=1:nc
            %%centroid perturbation
            E1=[0,0.001];
            C(i,:)=C(i,:)+E1;
            C(i+nc,:)=C(i,:)-E1;
        end
        nc=nc*2; %%increase the number of regions
        [idX,C]=divide(X,C,nc,k); %divide the regions
    end
end
