# @UnusedImport
from duckietown_utils.cli import D8App, d8app_run
from easy_algo.algo_db import get_easy_algo_db
from easy_algo.formatting import format_db, format_instances


__all__ = ['Summary']


class Summary(D8App):
    """ Provides information about the families and their instances. """

    def define_program_options(self, params):
        params.accept_extra()
        g = "Input/output"
        params.add_flag('verbose', short='-v', help='Verbose output', group=g)

    def go(self):
        args = self.options.get_extra()
        db = get_easy_algo_db()
        colorize = True
        verbose = self.options.verbose
        if len(args) == 0:
            s = format_db(db, verbose=verbose, colorize=colorize)

        elif len(args) == 1:
            family = db.get_family(args[0])

            s = format_instances(family, colorize=colorize,
                                 verbose=self.options.verbose)
        elif len(args) == 2:
            family_name = args[0]
            family = db.get_family(family_name)
            instance_name = args[1]
            instance = db.create_instance(family_name, instance_name)

            s = instance
        print(s)


if __name__ =='__main__':
    d8app_run(Summary)
