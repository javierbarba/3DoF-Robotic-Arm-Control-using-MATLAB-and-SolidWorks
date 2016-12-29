function [ DHinverse ] = DH_Inverse( DHmatrix )

DHinverse = [ DHmatrix(1:3,1:3).'  -DHmatrix(1:3,1:3).'*DHmatrix(1:3,4) ;
                [ 0  0  0 ]                          1                  ];

end