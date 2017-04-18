library(stats) # for bw.SJ() and density()
library(modeest) # for mlv()
library(mixtools) # for mvnormalmixEM()
# Assume data for sample i is in D
set.seed(1)
#mu=sample(-100:100, 9, replace=FALSE)/100
mu1=c(-0.47,-0.26,0.13,0.79,-0.61,0.76,0.84,0.28,0.21)
mu2=c(-0.88,-0.59,-0.65,0.36,-0.25,0.50,-0.03,0.39,0.91)
mu3=c(-0.24,0.55,0.86,-0.58,0.28,-0.76,-0.48,-0.26,-0.98)

mu=sample(-100:100, 9, replace=FALSE)/100

a<-rmvnorm(200,mu1)
b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
d<-density(a, bw=b, kernel="gaussian") # Gaussian kernel density estimate
d1<-d
plot(d,ylim=c(0, 0.6),lwd=3,col=10,lty=1,main="Driver 1")

for (i in c(1:9)){
  a<-rmvnorm(200,mu1[i])
  b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
  d<-density(a, bw=b, kernel="gaussian")
  lines(d,col=i,lwd=1,lty=i)
}

name=c('GMM','feature1','feature2','feature3','feature4','feature5'
       ,'feature6','feature7','feature8','feature9')
lwdline=c(3,1,1,1,1,1,1,1,1,1)
colline=c(10,1,2,3,4,5,6,7,8,9)
ltyline=c(1,1,2,3,4,5,6,7,8,9)
legend("topright", name,lwd=lwdline,col=colline,lty=ltyline)

a<-rmvnorm(200,mu2)
b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
d<-density(a, bw=b, kernel="gaussian") # Gaussian kernel density estimate
d2<-d
plot(d,ylim=c(0, 0.6),lwd=3,col=10,lty=1,main="Driver 2")

for (i in c(1:9)){
  a<-rmvnorm(200,mu2[i])
  b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
  d<-density(a, bw=b, kernel="gaussian")
  lines(d,col=i,lwd=1,lty=i)
}

name=c('GMM','feature1','feature2','feature3','feature4','feature5'
       ,'feature6','feature7','feature8','feature9')
lwdline=c(3,1,1,1,1,1,1,1,1,1)
colline=c(10,1,2,3,4,5,6,7,8,9)
ltyline=c(1,1,2,3,4,5,6,7,8,9)
legend("topright", name,lwd=lwdline,col=colline,lty=ltyline)

a<-rmvnorm(200,mu3)
b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
d<-density(a, bw=b, kernel="gaussian") # Gaussian kernel density estimate
d3<-d
plot(d,ylim=c(0, 0.6),lwd=3,col=10,lty=1,main="Driver 3")

for (i in c(1:9)){
  a<-rmvnorm(200,mu3[i])
  b<-bw.SJ(a) # Find bandwith with Shafer-Jones method
  d<-density(a, bw=b, kernel="gaussian")
  lines(d,col=i,lwd=1,lty=i)
}

name=c('GMM','feature1','feature2','feature3','feature4','feature5'
       ,'feature6','feature7','feature8','feature9')

lwdline=c(3,1,1,1,1,1,1,1,1,1)
colline=c(10,1,2,3,4,5,6,7,8,9)
ltyline=c(1,1,2,3,4,5,6,7,8,9)
legend("topright", name,lwd=lwdline,col=colline,lty=ltyline)


####merged
as<-rmvnorm(2000,c(0,0,0,0,0,0,0,0,0),sigma=diag(9))   #mu1 for driver 1's mu
b<-bw.SJ(as) # Find bandwith with Shafer-Jones method
d<-density(as, bw=b, kernel="gaussian") # Gaussian kernel density estimate
ds<-d
plot(ds,ylim=c(0, 0.6),lwd=3,col=1,lty=1,main="Comparison of Individual GMM and Standard GMM")
lines(d1,ylim=c(0, 0.6),lwd=3,col=2,lty=2)
lines(d2,ylim=c(0, 0.6),lwd=3,col=3,lty=3)
lines(d3,ylim=c(0, 0.6),lwd=3,col=4,lty=4)

name=c("Standard","Driver1","Driver2","Driver3")
colline=c(1,2,3,4)
ltyline=c(1,2,3,4)
legend("topright", name,col=colline,lty=ltyline)

#Driving Score with standard c(000000)
min.f1f2 <- function(x, mean1, mean2) {
  f1 <- rmvnorm(x, mean=mean1)
  f2 <- rmvnorm(x, mean=mean2)
  pmin(f1, f2)
}

integrate(min.f1f2, -Inf, Inf,mu1=c(0,0,0,0,0,0,0,0,0), mu2=mu1)
##############

min.f1f2 <- function(x, mu1_ex, mu2_ex) {
  f1 <- rmvnorm(x, mean=mu1_ex)
  f2 <- rmvnorm(x, mean=mu2_ex)
  pmin(f1, f2)
}

##Similarity

STEP=0.02
DX<-seq(-5, 5, STEP)

DY<-ds$y[0:length(DX)]

d1x<-d3$x
d1y<-d3$y

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


##### low score example

mulow<-c(-0.27,-0.13,-1.79,-1.73, 1.21,-1.64,0.59,-0.61,-1.51)
## safety score =0.6839662
