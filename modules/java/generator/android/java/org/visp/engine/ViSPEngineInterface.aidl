package org.visp.engine;

/**
* Class provides a Java interface for ViSP Engine Service. It's synchronous with native ViSPEngine class.
*/
interface ViSPEngineInterface
{
    /**
    * @return Returns service version.
    */
    int getEngineVersion();

    /**
    * Finds an installed ViSP library.
    * @param ViSP version.
    * @return Returns path to ViSP native libs or an empty string if ViSP can not be found.
    */
    String getLibPathByVersion(String version);

    /**
    * Tries to install defined version of ViSP from Google Play Market.
    * @param ViSP version.
    * @return Returns true if installation was successful or ViSP package has been already installed.
    */
    boolean installVersion(String version);

    /**
    * Returns list of libraries in loading order, separated by semicolon.
    * @param ViSP version.
    * @return Returns names of ViSP libraries, separated by semicolon.
    */
    String getLibraryList(String version);
}
