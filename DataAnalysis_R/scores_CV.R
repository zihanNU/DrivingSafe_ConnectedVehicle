mulow<-c(-0.27,-0.13,-1.79,-1.73, 1.21,-1.64,0.59,-0.61,-1.51)
## safety score =0.6839662
library(stats) # for bw.SJ() and density()
library(modeest) # for mlv()
library(mixtools) # for mvnormalmixEM()


min.f1f2 <-function(ds,d1){
  ##Similarity
  
  STEP=0.02
  DX<-seq(-5, 5, STEP)
  
  DY<-ds$y[0:length(DX)]
  
  d1x<-d1$x
  d1y<-d1$y
  
  for (i in c(2:500)){
    for (j in c(1:510)){
      if (DX[i]<d1x[1]){
        DY[i]=0
        break}
      if (DX[i]>d1x[length(d1x)])
      {DY[i]=0
      break}
      if ((DX[i]>d1x[j]) & (DX[i]<d1x[j+1])){
        k=(d1y[j+1]-d1y[j])/(d1x[j+1]-d1x[j])
        y=k*(DX[i]-d1x[j])+d1y[j]
        DY[i]=min(y,DY[i])
        break
      }
      if ((DX[i]>d1x[j]) & (DX[i]<d1x[j+2])){
        k=(d1y[j+2]-d1y[j])/(d1x[j+2]-d1x[j])
        y=k*(DX[i]-d1x[j])+d1y[j]
        DY[i]=min(y,DY[i])
        break
      }
    }
  }
  score<-sum(DY)*STEP
}


score_driver <-function(NUM,features){

  Scores<-rep(0, NUM)

  for (k in c(1:NUM)){
    #mu1=sample(-100:100, 9, replace=FALSE)/100
    mu1=features[k,]
    a<-rmvnorm(200,mu1)
    b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
    d<-density(a, bw=b, kernel="gaussian") # Gaussian kernel density estimate
    d1<-d
    Scores[k]=min.f1f2(ds,d1)
  }
  return (Scores)
}
features_0<-read.csv('Z:/Zihan/microsimulation/01052017/18000/R_analysis/driver_features_0.csv', header = F)
features_20<-read.csv('Z:/Zihan/microsimulation/01052017/18000/R_analysis/driver_features_20.csv', header = F)
features_score20<-read.csv('Z:/Zihan/microsimulation/01052017/18000/R_analysis/driver_features_20_all.csv', header = F)
features_10<-read.csv('Z:/Zihan/microsimulation/01052017/18000/R_analysis/driver_features_10.csv', header = F)


Scores0<-score_driver(2466,features_0)
Scores10<-score_driver(2466,features_10)
Scores20<-score_driver(2466,features_20)
Scores_CV20<-score_driver(2498,features_score20)

write.csv(Scores0,'Score0.csv')
write.csv(Scores20,'Score20.csv')
write.csv(Scores10,'Score10.csv')
write.csv(Scores_CV20,'Score_CV20_all.csv')


hist(Scores0*100,breaks=10,col=rgb(0.1,0.1,0.1,0.5),xlim=c(30,100),ylim=c(0,800))
hist(Scores20*100,breaks=10,col=rgb(0.0,0.1,0.1,0.5),xlim=c(30,100),ylim=c(0,800),add=T)
hist(Scores_CV20*100,col=rgb(0.8,0.8,0.8,0.5), add=T)
hist(Scores_scoreCV*100,breaks=10,add=T)
plot(density(Scores0),lwd=2,col=1,lty=1,xlim=c(0.6, 1.0))
lines(density(Scores10),lwd=2,col=2,lty=2)
lines(density(Scores_scoreCV),lwd=2,col=3,lty=3)

length(Scores_CV20[Scores_CV20 < 0.7])
length(Scores[Scores0 < 0.58])/2466
length(Scores[Scores0 < 0.60])/2466

datatest<-Scores_scoreCV
thre<-0.9
length(datatest[datatest < thre])

driver_ids<-rep(0, 89)
count=1
for (k in c(1:NUM)){
  if (Scores[k]<0.6){
    driver_ids[count]<-k
    count<-count+1
  }
}
# plot(d,ylim=c(0, 0.6),lwd=3,col=10,lty=1,main="Driver 1")
# 
# for (i in c(1:9)){
#   a<-rmvnorm(200,mu1[i])
#   b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
#   d<-density(a, bw=b, kernel="gaussian")
#   lines(d,col=i,lwd=1,lty=i)
# }
# 
# name=c('GMM','feature1','feature2','feature3','feature4','feature5'
#        ,'feature6','feature7','feature8','feature9')
# lwdline=c(3,1,1,1,1,1,1,1,1,1)
# colline=c(10,1,2,3,4,5,6,7,8,9)
# ltyline=c(1,1,2,3,4,5,6,7,8,9)
# legend("topright", name,lwd=lwdline,col=colline,lty=ltyline)




##Similarity

STEP=0.02
DX<-seq(-5, 5, STEP)

DY<-ds$y[0:length(DX)]

d1x<-d1$x
d1y<-d1$y

for (i in c(2:500)){
  for (j in c(1:510)){
    if (DX[i]<d1x[1]){
      DY[i]=0
      break}
    if (DX[i]>d1x[length(d1x)])
    {DY[i]=0
    break}
    if ((DX[i]>d1x[j]) & (DX[i]<d1x[j+1])){
      k=(d1y[j+1]-d1y[j])/(d1x[j+1]-d1x[j])
      y=k*(DX[i]-d1x[j])+d1y[j]
      DY[i]=min(y,DY[i])
      break
    }
    if ((DX[i]>d1x[j]) & (DX[i]<d1x[j+2])){
      k=(d1y[j+2]-d1y[j])/(d1x[j+2]-d1x[j])
      y=k*(DX[i]-d1x[j])+d1y[j]
      DY[i]=min(y,DY[i])
      break
    }
  }
}
score1<-sum(DY)*STEP
