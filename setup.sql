
/* TODO indicate dependency on flypush sql setup / compose in setup scripts */

/* TODO what to name table of only trajectories that are the same fly the whole
 * way through? flag? */
CREATE TABLE IF NOT EXISTS trajectories (
    tracking_pipeline_num smallint,
    run timestamptz(0) REFERENCES runs (run_timestamp),
    /* TODO also indicate whatever roi numbering scheme? */
    /* TODO TODO TODO calculate this, then make it part of pk, as opposed to
     * tracking pipeline num */
    roi_number smallint,
    /* TODO any reason to treat timeseries diff? (sep table, array, etc) */
    frame_times timestamptz[],
    trajectory path,
    /* TODO TODO check trajec and frame_times are same length */
    /* TODO also store area over time? other properties computed by tracker? */
    notes text,
    PRIMARY KEY(tracking_pipeline_num, run)
);


/* TODO define separately for diff chambers? (i.e. if slightly different
 * dimensions force different positioning under camera
   in general, how do i want to pick right ROIs for any given experiment?
   must need to know what kind of chamber + tracking was used, no?
 */
CREATE TABLE IF NOT EXISTS rois (
    /* TODO derive this from storing all rois that have been entered?
       what all roi shapes to support? just polygon for now? */
    centroid point
);

/* TODO store rig specific expected offsets of all rois? */


DELETE FROM flypush_tables WHERE table_name in ('trajectories', 'rois');

INSERT INTO flypush_tables (table_name, website_visible, website_editable)
    VALUES ('trajectories', FALSE, FALSE);

INSERT INTO flypush_tables (table_name, website_visible, website_editable)
    VALUES ('rois', FALSE, FALSE);
