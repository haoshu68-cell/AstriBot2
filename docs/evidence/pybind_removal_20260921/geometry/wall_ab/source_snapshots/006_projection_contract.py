"""Configuration evidence for the simulation projection, not a visibility proof."""
import math

SLICES=('low_obstacle','main_nav','torso_high','overhead')
PARAMETERS=('input_cloud_topic','base_frame','enable_outlier_filter')+tuple(
    f'slices.{name}.{field}' for name in SLICES for field in ('enabled','z_min','z_max','min_points'))

def projection_ceiling(values,base_frame):
    if values.get('input_cloud_topic')!='/map_scan' or values.get('base_frame')!=base_frame:
        raise ValueError('HEIGHT_PROJECTION_UNCLIPPED_SOURCE_REQUIRED')
    if values.get('enable_outlier_filter') is not False:
        raise ValueError('HEIGHT_PROJECTION_SPARSE_OBSTACLES_MAY_BE_REMOVED')
    end=-.03
    for name in SLICES:
        prefix='slices.'+name+'.'
        if values.get(prefix+'enabled') is not True or values.get(prefix+'min_points')!=1:
            raise ValueError('HEIGHT_PROJECTION_SLICE_INCOMPLETE')
        low,high=values.get(prefix+'z_min'),values.get(prefix+'z_max')
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (low,high)) or low>end+1e-9 or high<=low:
            raise ValueError('HEIGHT_PROJECTION_GAP_OR_INVALID_RANGE')
        end=high
    return end
