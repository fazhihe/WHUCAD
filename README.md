# WHUCAD

This repository provides source data for our paper:

[A parametric and feature-based CAD dataset to support human-computer interaction for advanced 3D shape learning](https://journals.sagepub.com/doi/full/10.3233/ICA-240744)

WHUCAD is the first full parametric and feature-based CAD dataset to support HCI in 3D learning, and therefore can support generating CAD models with real-world engineering features as advanced as that created by human engineers.

A gallery of WHUCAD dataset is shown above.

![image](https://github.com/fazhihe/WHUCAD/blob/main/A%20gallery%20of%20WHUCAD.png)

The vector format of WHUCAD is in the folder of ./data/vec . 

The link to download Mesh format of WHUCAD is https://gitee.com/fred926/whucad-mesh. 

The link to download Brep format of WHUCAD is https://gitee.com/fred926/whucad-brep.

The link to download Multiview format of WHUCAD is https://wdisk.whu.edu.cn/d/2cc15db07f7640b6b0e0. The password is: whucad250610

WHUCAD consists of two subsets. The first subset is derived from DeepCAD. We augment the data by randomly inserting sequences containing advanced features into DeepCAD data. The advantage of this approch is that it can obtain a rich amount of data with advanced features for learning and complex processing steps. We filter out unreasonable augmented data and check every model to ensure that WHUCAD has no redundant data. These models are in folders 0000-0099. The second subset comes from manual construction. We build 4,244 CAD models from the ABC dataset that have fruitful advanced features and semantic meaning but cannot be parsed into a command sequence by DeepCAD due to the lack of a selection mechanism. All models in the second subset of WHUCAD are macro recorded by CATIA and then parsed into vector sequence format. These models are in folders 0100-0103.

The citation of this paper is:

```
@article{ WOS:001360811000005,
Author = {Fan, Rubin and He, Fazhi and Liu, Yuxin and Song, Yupeng and Fan, Linkun
   and Yan, Xiaohu},
Title = {A parametric and feature-based CAD dataset to support human-computer
   interaction for advanced 3D shape learning},
Journal = {INTEGRATED COMPUTER-AIDED ENGINEERING},
Year = {2025},
Volume = {32},
Number = {1},
Pages = {73-94},
DOI = {10.3233/ICA-240744},
ISSN = {1069-2509},
EISSN = {1875-8835},
Unique-ID = {WOS:001360811000005},
}
```

## Prerequisites

- Linux
- NVIDIA GPU + CUDA CuDNN
- Python 3.7, PyTorch 1.5+

## Dependencies

Install python package dependencies through pip:

```bash
$ pip install -r requirements.txt
```

## Training

See all hyper-parameters and configurations under `config` folder. To train the autoencoder:

```bash
$ python train.py --exp_name WHUCAD -g 0
```

For random generation, further train a latent GAN:

```bash
# encode all data to latent space
$ python test.py --exp_name WHUCAD --mode enc --ckpt 1000 -g 0

# train latent GAN (wgan-gp)
$ python lgan.py --exp_name WHUCAD --ae_ckpt 1000 -g 0
```

The trained models and experment logs will be saved in `proj_log/WHUCAD/` by default.

## Testing and Evaluation

#### __Autoencoding__

  After training the autoencoder, run the model to reconstruct all test data:

```bash
$ python test.py --exp_name WHUCAD --mode rec --ckpt 1000 -g 0
```

The results will be saved in`proj_log/WHUCAD/results/test_1000` by default in the format of `h5` (CAD sequence saved in vectorized representation).

To evaluate the results:

```bash
$ cd evaluation
# for command accuray and parameter accuracy
$ python evaluate_ae_acc.py --src ../proj_log/WHUCAD/results/test_1000
```

#### __Random Generation__

  After training the latent GAN, run latent GAN and the autoencoder to do random generation:

```bash
# run latent GAN to generate fake latent vectors
$ python lgan.py --exp_name WHUCAD --ae_ckpt 1000 --ckpt 200000 --test --n_samples 9000 -g 0

# run the autoencoder to decode into final CAD sequences
$ python test.py --exp_name WHUCAD --mode dec --ckpt 1000 --z_path proj_log/WHUCAD/lgan_1000/results/fake_z_ckpt200000_num9000.h5 -g 0
```

The results will be saved in`proj_log/WHUCAD/lgan_1000/results` by default.

## Visualization and Export

We provide scripts to visualize CAD models in CATIA software.

## Acknowledgement

We would like to thank and acknowledge referenced codes from [DeepCAD: A Deep Generative Network for Computer-Aided Design Models](https://arxiv.org/abs/2105.09492).
