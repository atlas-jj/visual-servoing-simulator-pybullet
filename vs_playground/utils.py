import yaml


def configure(config_file):
    configs = None
    with open(config_file) as confs:
        try:
            configs = yaml.safe_load(confs)['simulation']
        except Exception as err:
            print('load configs failed! {}'.format(err))
            raise err
    return configs

