# Human-Joint-Constraints-Training
Training procedure for paper https://arxiv.org/abs/1709.08685. For usage during physics simulations, see https://github.com/dartsim/dart/pull/1016.

The Matlab procedure here depends on the database and code developed by Akhter and Black: http://poseprior.is.tue.mpg.de/.
You will need to sign up an account there to access their database. After you do that, unzip their prior.zip file, and move all the files in the unzipped folder into the Human-Joint-Constraints-Training folder.

Run toa_generate_random_pairs.m to generate samples for training the range(validity) of left arm poses;
Run tol_generate_random_pairs.m to generate samples for training the range(validity) of left leg & foot poses;

Neural-net training follows the standard process of feed-forward nets. There is an example python script using the Keras library for your reference.
