For multiple drones, no obstacles

We need to use neighborEmbedder before networkEvaluate

Example shows below:
===========
int main(const float *self_indatav, const float *nbr_indatav, float *outdatav){
    size_t i;
    control_t_n motorThrusts;

    neighborEmbedder(nbr_indatav);
    
    networkEvaluate(&motorThrusts, self_indatav);

    outdatav[0] = motorThrusts.thrust_0;
    outdatav[1] = motorThrusts.thrust_1;
    outdatav[2] = motorThrusts.thrust_2;
    outdatav[3] = motorThrusts.thrust_3;
    return EXIT_SUCCESS;
}
===========