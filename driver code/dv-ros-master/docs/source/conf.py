# Configuration file for the Sphinx documentation builder.

import os.path

# -- Project information

if os.path.isfile('version.py'):
    exec(open('version.py').read())
else:
    raise RuntimeError('Version information not found')

project = project_name
copyright = '2022, iniVation AG'
author = 'iniVation AG'

version = current_version
release = current_version

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.intersphinx',
    'sphinx_sitemap',
    'sphinx_rtd_theme',
    'myst_parser',
    'breathe',
]

templates_path = ['_templates']
html_static_path = ['_static']

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}

intersphinx_disabled_domains = ['std']

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

autosectionlabel_prefix_document = True

# -- Options for HTML output

html_css_files = ['inivation.css']

html_copy_source = False

html_theme_options = {
    'collapse_navigation': True,
    'navigation_depth': 2,
    'includehidden': False,
}

html_theme = 'sphinx_rtd_theme'

try:
    html_context
except NameError:
    html_context = dict()

html_context['current_version'] = current_version
html_context['version'] = current_version

html_context['downloads'] = list()
html_context['downloads'].append(('PDF', '%s-%s.pdf' % (project, current_version)))
html_context['downloads'].append(('ePub', '%s-%s.epub' % (project, current_version)))

# -- Options for EPUB output

epub_show_urls = 'footnote'

# -- Options for PDF output

latex_engine = 'pdflatex'
latex_show_urls = 'footnote'

# -- Options for Breathe Doxygen integration

breathe_projects = {
    project: '../doxy-xml/',
}
breathe_default_project = project
breathe_domain_by_extension = {
    'h': 'cpp',
    'hpp': 'cpp',
}
