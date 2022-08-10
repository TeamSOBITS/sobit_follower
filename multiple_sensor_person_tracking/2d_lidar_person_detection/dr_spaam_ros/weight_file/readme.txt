Namings:

ann  = annotations (the network is trained using annotation)
pl   = pseudo-labels (the network is trained using pseudo-labels)
ft   = fine-tuning (the network is fine-tuned from a pre-trained model on DROW)

phce  = partially-huberised cross entropy loss
mixup = mixup regularization


For deployment, take any checkpoint that is trained with annotations (i.e. has "ann" in its name).