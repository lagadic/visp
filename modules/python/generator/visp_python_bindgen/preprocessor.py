
# '''
# Preprocessor, derived from the command line preprocesor provided at https://github.com/ned14/pcpp/blob/master/pcpp/pcmd.py

# '''
# from __future__ import generators, print_function, absolute_import, division

# import sys, argparse, traceback, os, copy, io, re
# from pcpp.preprocessor import Preprocessor, OutputDirective, Action
# from visp_python_bindgen.generator_config import PreprocessorConfig



# class CmdPreprocessor(Preprocessor):
#     def __init__(self, config: PreprocessorConfig, input: str):
#         if len(argv) < 2:
#             argv = [argv[0], '--help']
#         argp = argparse.ArgumentParser(prog='pcpp',
#             description=
#     '''A pure universal Python C (pre-)preprocessor implementation very useful for
#     pre-preprocessing header only C++ libraries into single file includes and
#     other such build or packaging stage malarky.''',
#             epilog=
#     '''Note that so pcpp can stand in for other preprocessor tooling, it
#     ignores any arguments it does not understand.''')
#         argp.add_argument('-o', dest = 'output', metavar = 'path', type = argparse.FileType('wt'), default=sys.stdout, nargs = '?', help = 'Output to a file instead of stdout')
#         argp.add_argument('-D', dest = 'defines', metavar = 'macro[=val]', nargs = 1, action = 'append', help = 'Predefine name as a macro [with value]')
#         argp.add_argument('-U', dest = 'undefines', metavar = 'macro', nargs = 1, action = 'append', help = 'Pre-undefine name as a macro')
#         argp.add_argument('-N', dest = 'nevers', metavar = 'macro', nargs = 1, action = 'append', help = 'Never define name as a macro, even if defined during the preprocessing.')
#         argp.add_argument('-I', dest = 'includes', metavar = 'path', nargs = 1, action = 'append', help = "Path to search for unfound #include's")
#         #argp.add_argument('--passthru', dest = 'passthru', action = 'store_true', help = 'Pass through everything unexecuted except for #include and include guards (which need to be the first thing in an include file')
#         argp.add_argument('--passthru-defines', dest = 'passthru_defines', action = 'store_true', help = 'Pass through but still execute #defines and #undefs if not always removed by preprocessor logic')
#         argp.add_argument('--passthru-unfound-includes', dest = 'passthru_unfound_includes', action = 'store_true', help = 'Pass through #includes not found without execution')
#         argp.add_argument('--passthru-unknown-exprs', dest = 'passthru_undefined_exprs', action = 'store_true', help = 'Unknown macros in expressions cause preprocessor logic to be passed through instead of executed by treating unknown macros as 0L')
#         argp.add_argument('--passthru-comments', dest = 'passthru_comments', action = 'store_true', help = 'Pass through comments unmodified')
#         argp.add_argument('--passthru-magic-macros', dest = 'passthru_magic_macros', action = 'store_true', help = 'Pass through double underscore magic macros unmodified')
#         argp.add_argument('--passthru-includes', dest = 'passthru_includes', metavar = '<regex>', default = None, nargs = 1, help = "Regular expression for which #includes to not expand. #includes, if found, are always executed")
#         argp.add_argument('--line-directive', dest = 'line_directive', metavar = 'form', default = '#line', nargs = '?', help = "Form of line directive to use, defaults to #line, specify nothing to disable output of line directives")
#         args = argp.parse_known_args(argv[1:])
#         #print(args)
#         for arg in args[1]:
#             print("NOTE: Argument %s not known, ignoring!" % arg, file = sys.stderr)

#         self.args = args[0]
#         super(CmdPreprocessor, self).__init__()

#         # Override Preprocessor instance variables
#         self.define("__PCPP_ALWAYS_FALSE__ 0")
#         self.define("__PCPP_ALWAYS_TRUE__ 1")

#         self.auto_pragma_once_enabled = True
#         self.line_directive = config.line_directive
#         if self.line_directive is not None and self.line_directive.lower() in ('nothing', 'none', ''):
#             self.line_directive = None
#         self.passthru_includes = re.compile(config.passthrough_includes_regex)
#         self.compress = 0
#         # Pass through magic macros
#         if False:
#             self.undef('__DATE__')
#             self.undef('__TIME__')
#             self.expand_linemacro = False
#             self.expand_filemacro = False
#             self.expand_countermacro = False

#         # My own instance variables
#         self.bypass_ifpassthru = False
#         self.potential_include_guard = None


#         for d in config.defines:
#             if '=' not in d:
#                 d += '=1'
#             d = d.replace('=', ' ', 1)
#             self.define(d)
#         # for d in config.undefines:
#         #     self.undef(d)
#         self.nevers = config.never_defined
#         if self.args.nevers:
#             self.args.nevers = [x[0] for x in self.args.nevers]

#         for include in config.include_directories:
#             self.add_path(include)


#         try:
#             if len(self.args.inputs) == 1:
#                 self.parse(self.args.inputs[0])
#             else:
#                 input = ''
#                 for i in self.args.inputs:
#                     input += '#include "' + i.name + '"\n'
#                 self.parse(input)
#             self.write(self.args.output)
#         except:
#             print(traceback.print_exc(10), file = sys.stderr)
#             print("\nINTERNAL PREPROCESSOR ERROR AT AROUND %s:%d, FATALLY EXITING NOW\n"
#                 % (self.lastdirective.source, self.lastdirective.lineno), file = sys.stderr)
#             sys.exit(-99)
#         finally:
#             for i in self.args.inputs:
#                 i.close()
#             if self.args.output != sys.stdout:
#                 self.args.output.close()



#     def on_include_not_found(self,is_malformed,is_system_include,curdir,includepath):
#         if self.args.passthru_unfound_includes:
#             raise OutputDirective(Action.IgnoreAndPassThrough)
#         return super(CmdPreprocessor, self).on_include_not_found(is_malformed,is_system_include,curdir,includepath)

#     def on_unknown_macro_in_defined_expr(self,tok):
#         if self.args.undefines:
#             if tok.value in self.args.undefines:
#                 return False
#         if self.args.passthru_undefined_exprs:
#             return None  # Pass through as expanded as possible
#         return super(CmdPreprocessor, self).on_unknown_macro_in_defined_expr(tok)

#     def on_unknown_macro_in_expr(self,ident):
#         if self.args.undefines:
#             if ident in self.args.undefines:
#                 return super(CmdPreprocessor, self).on_unknown_macro_in_expr(ident)
#         if self.args.passthru_undefined_exprs:
#             return None  # Pass through as expanded as possible
#         return super(CmdPreprocessor, self).on_unknown_macro_in_expr(ident)

#     def on_unknown_macro_function_in_expr(self,ident):
#         if self.args.undefines:
#             if ident in self.args.undefines:
#                 return super(CmdPreprocessor, self).on_unknown_macro_function_in_expr(ident)
#         if self.args.passthru_undefined_exprs:
#             return None  # Pass through as expanded as possible
#         return super(CmdPreprocessor, self).on_unknown_macro_function_in_expr(ident)

#     def on_directive_handle(self,directive,toks,ifpassthru,precedingtoks):
#         if ifpassthru:
#             if directive.value == 'if' or directive.value == 'elif' or directive == 'else' or directive.value == 'endif':
#                 self.bypass_ifpassthru = len([tok for tok in toks if tok.value == '__PCPP_ALWAYS_FALSE__' or tok.value == '__PCPP_ALWAYS_TRUE__']) > 0
#             if not self.bypass_ifpassthru and (directive.value == 'define' or directive.value == 'undef'):
#                 if toks[0].value != self.potential_include_guard:
#                     raise OutputDirective(Action.IgnoreAndPassThrough)  # Don't execute anything with effects when inside an #if expr with undefined macro
#         if (directive.value == 'define' or directive.value == 'undef') and self.args.nevers:
#             if toks[0].value in self.args.nevers:
#                 raise OutputDirective(Action.IgnoreAndPassThrough)
#         if self.args.passthru_defines:
#             super(CmdPreprocessor, self).on_directive_handle(directive,toks,ifpassthru,precedingtoks)
#             return None  # Pass through where possible
#         return super(CmdPreprocessor, self).on_directive_handle(directive,toks,ifpassthru,precedingtoks)

#     def on_directive_unknown(self,directive,toks,ifpassthru,precedingtoks):
#         if ifpassthru:
#             return None  # Pass through
#         return super(CmdPreprocessor, self).on_directive_unknown(directive,toks,ifpassthru,precedingtoks)

#     def on_potential_include_guard(self,macro):
#         self.potential_include_guard = macro
#         return super(CmdPreprocessor, self).on_potential_include_guard(macro)

#     def on_comment(self,tok):
#         if self.args.passthru_comments:
#             return True  # Pass through
#         return super(CmdPreprocessor, self).on_comment(tok)

# def main(argv=None):
#     if argv is None:
#         argv = sys.argv
#     p = CmdPreprocessor(argv)
#     return p.return_code

# if __name__ == "__main__":
#     sys.exit(main(sys.argv))
